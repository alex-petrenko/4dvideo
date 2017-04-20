#include <queue>
#include <cassert>
#include <fstream>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <tri/triangulation.hpp>

#include <util/geometry.hpp>
#include <util/tiny_logger.hpp>


class Delaunay::DelaunayImpl
{
    friend class Delaunay;

private:
    DelaunayImpl()
        : totalNumPoints(0)
        , numEdges(0)
    {
    }

    // Geometry helpers.

    /// p1,p2,p3 should be oriented counterclockwise!
    FORCE_INLINE bool inCircle(uint16_t p1, uint16_t p2, uint16_t p3, uint16_t p)
    {
        return ::inCircle(P[p1].j, P[p1].i, P[p2].j, P[p2].i, P[p3].j, P[p3].i, P[p].j, P[p].i);
    }

    FORCE_INLINE Orientation orientationPointEdge(uint16_t p, const TriEdge &e)
    {
        const uint16_t orig = e.origPnt, dest = destPnt(e);
        return triOrientation(P[orig].j, P[orig].i, P[dest].j, P[dest].i, P[p].j, P[p].i);
    }

    FORCE_INLINE bool isRightOf(uint16_t p, EdgeIdx e)
    {
        return orientationPointEdge(p, E[e]) == ORIENT_CW;
    }

    FORCE_INLINE bool isLeftOf(uint16_t p, EdgeIdx e)
    {
        return orientationPointEdge(p, E[e]) == ORIENT_CCW;
    }


    // Topology helpers.

    /// Same edge but in opposite direction.
    FORCE_INLINE TriEdge & sym(const TriEdge &e)
    {
        return E[e.symEdge];
    }

    FORCE_INLINE TriEdge & sym(EdgeIdx edgeIdx)
    {
        return E[E[edgeIdx].symEdge];
    }

    /// Destination point of an edge.
    FORCE_INLINE uint16_t destPnt(const TriEdge &e)
    {
        return E[e.symEdge].origPnt;
    }

    FORCE_INLINE uint16_t destPnt(EdgeIdx edgeIdx)
    {
        return E[E[edgeIdx].symEdge].origPnt;
    }

    /// Creates pair of oriented edges orig->dest and dest->orig.
    FORCE_INLINE EdgeIdx makeEdge(uint16_t orig, uint16_t dest)
    {
        const EdgeIdx e1Idx = numEdges, e2Idx = e1Idx + 1;
        numEdges += 2;
        assert(numEdges < maxNumEdges / 2);

        TriEdge &e1 = E[e1Idx];
        TriEdge &e2 = E[e2Idx];

        e1.origPnt = orig;
        e2.origPnt = dest;

        e1.symEdge = e2Idx;
        e2.symEdge = e1Idx;

        e1.nextCcwEdge = e1.prevCcwEdge = e1Idx;
        e2.nextCcwEdge = e2.prevCcwEdge = e2Idx;

        return e1Idx;
    }

    /// Attach newEdge to the origin of oldEdge immediately after oldEdge counterclockwise.
    FORCE_INLINE void join(EdgeIdx newEdgeIdx, EdgeIdx oldEdgeIdx)
    {
        TriEdge &newEdge = E[newEdgeIdx];
        TriEdge &oldEdge = E[oldEdgeIdx];

        assert(newEdge.origPnt == oldEdge.origPnt);

        newEdge.nextCcwEdge = oldEdge.nextCcwEdge;
        oldEdge.nextCcwEdge = newEdgeIdx;

        E[newEdge.nextCcwEdge].prevCcwEdge = newEdgeIdx;
        newEdge.prevCcwEdge = oldEdgeIdx;
    }

    /// Connects destination of a to origin of b. Returns e = a.dest->b.orig.
    FORCE_INLINE EdgeIdx connect(EdgeIdx aIdx, EdgeIdx bIdx)
    {
        const TriEdge &b = E[bIdx];

        const EdgeIdx cIdx = makeEdge(destPnt(aIdx), b.origPnt);
        const TriEdge &c = E[cIdx];
        join(cIdx, sym(aIdx).prevCcwEdge);
        join(c.symEdge, bIdx);

        assert(destPnt(aIdx) == c.origPnt);
        assert(b.origPnt == destPnt(c));
        assert(destPnt(aIdx) == destPnt(sym(c)));
        assert(b.origPnt == sym(c).origPnt);
        return cIdx;
    }

    /// Removes references to given edge from the linked list around it's origin.
    FORCE_INLINE void unlink(EdgeIdx eIdx)
    {
        const TriEdge &e = E[eIdx];
        E[e.nextCcwEdge].prevCcwEdge = e.prevCcwEdge;
        E[e.prevCcwEdge].nextCcwEdge = e.nextCcwEdge;
    }

    /// Removes given edge and its pair from the subdivision.
    FORCE_INLINE void deleteEdge(EdgeIdx eIdx)
    {
        unlink(eIdx);
        unlink(E[eIdx].symEdge);
    }


    // Main algorithm.

    /// Call before each run of an algorithm.
    void init(const std::vector<PointIJ> &points);

    /// Very fast radix sort.
    void sortPoints(std::vector<PointIJ> &points, std::vector<short> &indexMap);

    /// Algorithm entry point.
    void triangulate();

    /// Subtask.
    void triangulateSubset(uint16_t lIdx, uint16_t numPoints, EdgeIdx &le, EdgeIdx &re);

    /// Merge phase. Joining left and right triangulations into one.
    FORCE_INLINE void mergeTriangulations(EdgeIdx lle, EdgeIdx lre, EdgeIdx rle, EdgeIdx rre, EdgeIdx &le, EdgeIdx &re);

    // Read results.

    /// Returns number of triangles and pointer to the triangle array.
    void getTriangles(Triangle *&t, int &num)
    {
        t = triangles;
        num = numTriangles;
    }

    // Auxiliary stuff.

    /// Draw connected component into a given cv::Mat.
    /// Highlights leftmost, rightmost and base edges if they are given.
    void plotTriangulation(cv::Mat &img, EdgeIdx le, EdgeIdx re = INVALID_EDGE, EdgeIdx base = INVALID_EDGE);

    /// Show triangulation on a screen (mostly for debugging purposes).
    void showTriangulation(cv::Mat &img, EdgeIdx le, EdgeIdx re = INVALID_EDGE, EdgeIdx base = INVALID_EDGE);

    /// Obtain list of triangles from the triangulation graph.
    void generateTriangles();

    /// Save triangulation structure into file.
    static void saveTriangulation(const std::string &filename, int numP, const PointIJ *p, int numT, const Triangle *t);

    /// Load from file. Used for testing.
    static void loadTriangulation(const std::string &filename, std::vector<PointIJ> &p, std::vector<Triangle> &t);

    /// Returns true if triangulation represents the same connectivity as given arrays of points and triangles.
    bool isEqualTo(const std::vector<PointIJ> &p, const std::vector<Triangle> &t) const;

private:
    uint16_t totalNumPoints = 0;
    const PointIJ *P;

    int numEdges = 0;
    TriEdge E[maxNumEdges];
    EdgeIdx leftmostEdge = INVALID_EDGE, rightmostEdge = INVALID_EDGE;  // valid only when triangulation is calculated

    int numTriangles = 0;
    Triangle triangles[maxNumTriangles];

    // auxiliary data for generateTriangles
    EdgeIdx bfsQueue[maxNumEdges];
    bool bfsVisited[maxNumEdges];

    // auxiliary data for radix sort
    PointIJ pointsSortedByI[maxNumPoints];
    short originalIdx[maxNumPoints];
    short numPointsPerCoord[maxCoord], coordIdx[maxCoord];
};


/// Accepts a sorted array of points without duplicates.
void Delaunay::DelaunayImpl::init(const std::vector<PointIJ> &points)
{
    assert(sizeof(TriEdge) <= 16);
    P = points.data();
    assert(points.size() < std::numeric_limits<uint16_t>::max());
    totalNumPoints = uint16_t(points.size());
    numEdges = numTriangles = 0;
}

/// Algorithm only works with sorted points, so this function is used to sort them. Also removes duplicates from the sequence.
/// Returned index map contains original index for every point in sorted sequence.
void Delaunay::DelaunayImpl::sortPoints(std::vector<PointIJ> &points, std::vector<short> &indexMap)
{
    assert(points.size() < std::numeric_limits<short>::max() - 1);
    assert(indexMap.size() == points.size());

    // Doing something like radix sort (1st - counting sort by i coord, then counting sort by j coord).
    // This is about 10x faster than std::sort.
    memset(numPointsPerCoord, 0, sizeof(numPointsPerCoord));
    memset(coordIdx, 0, sizeof(coordIdx));
    for (size_t i = 0; i < points.size(); ++i)
        ++numPointsPerCoord[points[i].i];
    // sort in reverse order by i coordinate
    for (int i = maxCoord - 2; i >= 0; --i)
        coordIdx[i] = coordIdx[i + 1] + numPointsPerCoord[i + 1];
    for (size_t i = 0; i < points.size(); ++i)
    {
        const PointIJ &point = points[i];
        const short newIdx = coordIdx[point.i];
        pointsSortedByI[newIdx] = point;
        originalIdx[newIdx] = short(i);
        ++coordIdx[point.i];
        assert(coordIdx[point.i] <= points.size());
    }

    memset(numPointsPerCoord, 0, sizeof(numPointsPerCoord));
    memset(coordIdx, 0, sizeof(coordIdx));
    for (size_t i = 0; i < points.size(); ++i)
        ++numPointsPerCoord[points[i].j];
    for (int i = 1; i < maxCoord; ++i)
        coordIdx[i] = coordIdx[i - 1] + numPointsPerCoord[i - 1];
    for (int i = 0; i < points.size(); ++i)
    {
        const PointIJ &point = pointsSortedByI[i];
        const short newIdx = coordIdx[point.j];
        points[newIdx] = point;
        indexMap[newIdx] = originalIdx[i];
        ++coordIdx[point.j];
        assert(newIdx <= points.size());
    }

    // Removes all duplicates from the sequence, analogue of std::unique.
    // Thanks to outer loop it works very fast if there are no duplicates in the sequence.
    {
        int newSize = int(points.size());
        for (int left = 0, right = 1; right < points.size(); ++left, ++right)
        {
            if (points[left] == points[right])
            {
                for (; right < points.size(); ++right)
                {
                    if (points[left] != points[right])
                    {
                        ++left;
                        assert(left != right);
                    }

                    points[left] = points[right];
                    indexMap[left] = indexMap[right];
                }

                assert(left < points.size());
                assert(right <= points.size());
                newSize = left + 1;
                break;
            }
        }

        points.resize(newSize);
        indexMap.resize(newSize);
    }
}

/// Simple wrapper that calls triangulateSubset for the whole set of points.
/// Returns the leftmost edge of the subdivision convex hull.
void Delaunay::DelaunayImpl::triangulate()
{
    if (totalNumPoints < 2)
    {
        TLOG(INFO) << "Cannot triangulate set of " << totalNumPoints << " points";
        return;
    }

    leftmostEdge = rightmostEdge = INVALID_EDGE;
    triangulateSubset(0, totalNumPoints, leftmostEdge, rightmostEdge);
}

/// Merge phase.
FORCE_INLINE void Delaunay::DelaunayImpl::mergeTriangulations(EdgeIdx lle, EdgeIdx lre, EdgeIdx rle, EdgeIdx rre, EdgeIdx &le, EdgeIdx &re)
{
    // first, find the new base edge, the lower common tangent of left and right subdivisions.
    while (true)
    {
        // move edges down across convex hull of subdivision until we find a tangent
        if (isLeftOf(E[rle].origPnt, lre))
        {
            // best right candidate for the tangent endpoint still "below", move lre further down
            lre = sym(lre).prevCcwEdge;
        }
        else if (isRightOf(E[lre].origPnt, rle))
        {
            // best left candidate for the tangent endpoint still "below", move rle further down
            rle = sym(rle).nextCcwEdge;
        }
        else
        {
            // tangent endpoints are found
            break;
        }
    }

    // create "base" edge, we will work up from it merging the triangulations
    EdgeIdx base = connect(E[rle].symEdge, lre);
    assert(!isLeftOf(E[lle].origPnt, base));
    assert(!isLeftOf(E[rre].origPnt, base));

    // it may be required to update le and re
    if (E[lle].origPnt == destPnt(base))
        lle = E[base].symEdge;
    if (E[rre].origPnt == E[base].origPnt)
        rre = base;

    le = lle, re = rre;

    // Main merging code. Find left and right candidate and update base edge to move up.
    // When no candidates are left we reached the convex hull.
    // For more information read: http://www.sccg.sk/~samuelcik/dgs/quad_edge.pdf page 114.
    while (true)
    {
        EdgeIdx lCand = sym(base).nextCcwEdge;
        bool lCandFound = false;
        while (isRightOf(destPnt(lCand), base))
        {
            const EdgeIdx nextLCand = E[lCand].nextCcwEdge;
            if (inCircle(destPnt(base), E[base].origPnt, destPnt(lCand), destPnt(nextLCand)))
            {
                assert(isRightOf(destPnt(nextLCand), base));
                deleteEdge(lCand);
                lCand = nextLCand;
            }
            else
            {
                // left candidate is found, we can move on
                lCandFound = true;
                break;
            }
        }

#if ALTERNATIVE_MERGE
        EdgeIdx lCand = sym(base).nextCcwEdge;
        bool lCandFound = false;
        if (isRightOf(destPnt(lCand), base))
        {
            EdgeIdx nextLCand = E[lCand].nextCcwEdge;
            // showTriangulation(triangImg, lCand, INVALID_EDGE, base);
            while (inCircle(destPnt(base), E[base].origPnt, destPnt(lCand), destPnt(nextLCand)))
            {
                assert(isRightOf(destPnt(nextLCand), base));
                deleteEdge(lCand);
                lCand = nextLCand;
                nextLCand = E[lCand].nextCcwEdge;
                // showTriangulation(triangImg, lCand, INVALID_EDGE, base);
            }
            assert(isRightOf(destPnt(lCand), base));
            // left candidate is found, we can move on
            lCandFound = true;
        }
#endif

        EdgeIdx rCand = E[base].prevCcwEdge;
        bool rCandFound = false;
        while (isRightOf(destPnt(rCand), base))
        {
            const EdgeIdx nextRCand = E[rCand].prevCcwEdge;
            // showTriangulation(triangImg, INVALID_EDGE, rCand, base);
            if (inCircle(destPnt(base), E[base].origPnt, destPnt(rCand), destPnt(nextRCand)))
            {
                deleteEdge(rCand);
                rCand = nextRCand;
                // showTriangulation(triangImg, INVALID_EDGE, rCand, base);
            }
            else
            {
                // right candidate is found, we can move on
                rCandFound = true;
                break;
            }
        }

#if ALTERNATIVE_MERGE
        EdgeIdx rCand = E[base].prevCcwEdge;
        bool rCandFound = false;
        if (isRightOf(destPnt(rCand), base))
        {
            EdgeIdx nextRCand = E[rCand].prevCcwEdge;
            // showTriangulation(triangImg, INVALID_EDGE, rCand, base);
            while (inCircle(destPnt(base), E[base].origPnt, destPnt(rCand), destPnt(nextRCand)))
            {
                assert(isRightOf(destPnt(nextRCand), base));
                deleteEdge(rCand);
                rCand = nextRCand;
                nextRCand = E[rCand].prevCcwEdge;
                // showTriangulation(triangImg, INVALID_EDGE, rCand, base);
            }
            assert(isRightOf(destPnt(rCand), base));
            // right candidate is found, we can move on
            rCandFound = true;
        }
#endif

        if (!lCandFound && !rCandFound)
        {
            // cannot continue updating base edge, this should mean that we've reached upper common tangent of two subdivisions
            {
                // all triangulation should now be to the left of the base edge, let's do a couple of checks just in case
                assert(!isRightOf(E[lle].origPnt, base));
                assert(!isRightOf(E[lre].origPnt, base));
                assert(!isRightOf(E[rle].origPnt, base));
                assert(!isRightOf(E[rre].origPnt, base));
            }

            break;
        }
        else  // at least one valid candidate is found
        {
            if (!lCandFound || (rCandFound && inCircle(destPnt(base), E[base].origPnt, destPnt(lCand), destPnt(rCand))))
            {
                // either lCand not found or rCand destination is in lCand triangle circumcircle --> select rCand
                base = connect(rCand, E[base].symEdge);
            }
            else
            {
                // select lCand
                base = connect(E[base].symEdge, E[lCand].symEdge);
            }
        }
    }
}

/// Main divide and conquer algorithm.
/// Each call modifies the subdivision structure and returns two edges from the convex hull.
/// le - index of CCW convex hull edge from leftmost vertex
/// re - index of CW convex hull edge from rightmost vertex
void Delaunay::DelaunayImpl::triangulateSubset(uint16_t lIdx, uint16_t numPoints, EdgeIdx &le, EdgeIdx &re)
{
    assert(numPoints < maxNumPoints);

    if (numPoints == 2)
    {
        const uint16_t s1 = lIdx, s2 = s1 + 1;
        le = makeEdge(s1, s2);
        re = E[le].symEdge;
    }
    else if (numPoints == 3)
    {
        const uint16_t s1 = lIdx, s2 = s1 + 1, s3 = s2 + 1;
        const EdgeIdx aIdx = makeEdge(s1, s2);
        const EdgeIdx bIdx = makeEdge(s2, s3);

        join(E[aIdx].symEdge, bIdx);  // now a.sym and b are adjacent

        const Orientation pos = triOrientation(P[s1].j, P[s1].i, P[s2].j, P[s2].i, P[s3].j, P[s3].i);
        switch (pos)
        {
        case ORIENT_CCW:
            connect(bIdx, aIdx);
            le = aIdx;
            re = E[bIdx].symEdge;
            break;
        case ORIENT_CW:
            re = connect(bIdx, aIdx);
            le = E[re].symEdge;
            break;
        default:
            // points are collinear
            le = aIdx;
            re = E[bIdx].symEdge;
            break;
        }
    }
    else
    {
        assert(numPoints >= 4);

        const uint16_t numRight = numPoints / 2, numLeft = numPoints - numRight;
        EdgeIdx lle;  // index of CCW convex hull edge from leftmost vertex of left triangulation
        EdgeIdx lre;  // index of CW convex hull edge from rightmost vertex of left triangulation
        EdgeIdx rle;  // index of CCW convex hull edge from leftmost vertex of right triangulation
        EdgeIdx rre;  // index of CW convex hull edge from rightmost vertex of right triangulation
        triangulateSubset(lIdx + numLeft, numRight, rle, rre);
        triangulateSubset(lIdx, numLeft, lle, lre);

        mergeTriangulations(lle, lre, rle, rre, le, re);
    }
}

#if USE_NON_RECURSIVE_TRIANGULATION

const int maxStackSize = 100;
int stackIdx = 0;

enum class Stage
{
    START = 0,
    SUBTASKS_CALLED = 1,
};

struct TriangulationParams
{
    EdgeIdx lle, lre, rle, rre;
    Stage stage;
    uint16_t lIdx, numPoints;
    EdgeIdx *le, *re;  // return values
};

TriangulationParams stack[maxStackSize];


void triangulateNonRecursive(uint16_t initLIdx, uint16_t initNumPoints, EdgeIdx &leReturn, EdgeIdx &reReturn)
{
    assert(initNumPoints < maxNumPoints);

    TriangulationParams &init = stack[0];
    init.lIdx = initLIdx;
    init.numPoints = initNumPoints;
    init.stage = Stage::START;
    init.le = &leReturn;
    init.re = &reReturn;

    while (stackIdx >= 0)
    {
        assert(stackIdx < maxStackSize / 2);

        TriangulationParams &params = stack[stackIdx];
        const uint16_t numPoints = params.numPoints;
        const uint16_t lIdx = params.lIdx;
        EdgeIdx * const le = params.le;
        EdgeIdx * const re = params.re;

        if (params.stage == Stage::START)
        {
            if (numPoints == 2)
            {
                const uint16_t s1 = lIdx, s2 = s1 + 1;
                *le = makeEdge(s1, s2);
                *re = E[*le].symEdge;
                --stackIdx;
            }
            else if (numPoints == 3)
            {
                const uint16_t s1 = lIdx, s2 = s1 + 1, s3 = s2 + 1;
                const EdgeIdx aIdx = makeEdge(s1, s2);
                const EdgeIdx bIdx = makeEdge(s2, s3);

                join(E[aIdx].symEdge, bIdx);  // now a.sym and b are adjacent

                const Orientation pos = triOrientation(P(s1).j, P(s1).i, P(s2).j, P(s2).i, P(s3).j, P(s3).i);
                switch (pos)
                {
                case ORIENT_CCW:
                    connect(bIdx, aIdx);
                    *le = aIdx;
                    *re = E[bIdx].symEdge;
                    break;
                case ORIENT_CW:
                    *re = connect(bIdx, aIdx);
                    *le = E[*re].symEdge;
                    break;
                default:
                    // points are collinear
                    *le = aIdx;
                    *re = E[bIdx].symEdge;
                    break;
                }

                --stackIdx;
            }
            else
            {
                assert(numPoints >= 4);

                const int numRight = numPoints / 2, numLeft = numPoints - numRight;

                TriangulationParams &pRight = stack[++stackIdx];
                pRight.stage = Stage::START;
                pRight.lIdx = lIdx + numLeft;
                pRight.numPoints = numRight;
                pRight.le = &params.rle;
                pRight.re = &params.rre;

                TriangulationParams &pLeft = stack[++stackIdx];
                pLeft.stage = Stage::START;
                pLeft.lIdx = params.lIdx;
                pLeft.numPoints = numLeft;
                pLeft.le = &params.lle;
                pLeft.re = &params.lre;

                params.stage = Stage::SUBTASKS_CALLED;
            }
        }
        else
        {
            assert(params.stage == Stage::SUBTASKS_CALLED);

            mergeTriangulations(params.lle, params.lre, params.rle, params.rre, *params.le, *params.re);
            --stackIdx;
        }
    }
}
#endif


void Delaunay::DelaunayImpl::generateTriangles()
{
    const EdgeIdx startEdge = leftmostEdge;
    assert(startEdge != INVALID_EDGE);

    memset(bfsVisited, 0, numEdges);
    numTriangles = 0;

    // triangulation is a single connected component, so we can just traverse
    // the whole structure collecting triangles along the way
    int queueHead = 0, queueTail = 0;
    bfsQueue[queueTail++] = startEdge;
    bfsVisited[startEdge] = true;

    while (queueTail != queueHead)
    {
        const EdgeIdx currEdgeIdx = bfsQueue[queueHead++];
        const TriEdge &currEdge = E[currEdgeIdx];
        const EdgeIdx symEdgeIdx = currEdge.symEdge;

        // Try to find triangle to the right hand of the edge.
        const EdgeIdx side1 = E[currEdge.prevCcwEdge].symEdge;
        const EdgeIdx side2 = E[symEdgeIdx].nextCcwEdge;

        assert(destPnt(side1) == currEdge.origPnt);
        assert(E[side2].origPnt == destPnt(currEdge));

        if (E[side1].origPnt == destPnt(side2))
        {
            // Three edges indeed form an oriented clockwise triangle.
            // Let's add it to the list of triangles! (TODO: triangle strips?)
            Triangle &t = triangles[numTriangles++];
            t.p1 = currEdge.origPnt;
            t.p2 = E[side1].origPnt;
            t.p3 = E[side2].origPnt;

            // Add symmetric edges to the queue, they may belong to adjacent triangles.
            const EdgeIdx side1Sym = E[side1].symEdge;
            const EdgeIdx side2Sym = E[side2].symEdge;
            if (!bfsVisited[side1Sym])
            {
                bfsQueue[queueTail++] = side1Sym;
                bfsVisited[side1Sym] = true;
            }
            if (!bfsVisited[side2Sym])
            {
                bfsQueue[queueTail++] = side2Sym;
                bfsVisited[side2Sym] = true;
            }

            // (Won't work as expected if triangulation has only one triangle. In this case
            // we will add it two times: CW and CCW. But it's not worth it to add additional checks for this
            // rare occasion).
        }

        // Add symmetric edge to the queue anyway, even if current edge does not belong to any triangle.
        if (!bfsVisited[symEdgeIdx])
        {
            bfsQueue[queueTail++] = symEdgeIdx;
            bfsVisited[symEdgeIdx] = true;
        }
    }
}

void Delaunay::DelaunayImpl::saveTriangulation(const std::string &filename, int numP, const PointIJ *p, int numT, const Triangle *t)
{
    std::ofstream tri(filename);
    tri << numP << '\n';
    for (int i = 0; i < numP; ++i)
        tri << p[i].i << ' ' << p[i].j << '\n';

    assert(numT);
    tri << numT << '\n';
    for (int i = 0; i < numT; ++i)
        tri << t[i].p1 << ' ' << t[i].p2 << ' ' << t[i].p3 << '\n';
}

void Delaunay::DelaunayImpl::loadTriangulation(const std::string &filename, std::vector<PointIJ> &p, std::vector<Triangle> &t)
{
    std::ifstream tri{ filename };
    int numP = 0, numT = 0;
    tri >> numP;
    p.resize(numP);
    for (int i = 0; i < numP; ++i)
        tri >> p[i].i >> p[i].j;

    tri >> numT;
    t.resize(numT);
    for (int i = 0; i < numT; ++i)
        tri >> t[i].p1 >> t[i].p2 >> t[i].p3;
}

/// Build explicit adjacent lists for each point, sort them and check if they are equal.
bool Delaunay::DelaunayImpl::isEqualTo(const std::vector<PointIJ> &points, const std::vector<Triangle> &tri) const
{
    if (points.size() != totalNumPoints)
    {
        TLOG(INFO) << "number of points is not equal";
        return false;
    }

    // points are sorted so we can compare them like this
    if (memcmp(P, points.data(), totalNumPoints * sizeof(points.front())))
    {
        TLOG(INFO) << "points are not equal";
        return false;
    }

    if (tri.size() != numTriangles)
    {
        TLOG(INFO) << "number of triangles is not equal";
        return false;
    }

    typedef std::vector<std::vector<uint16_t>> AdjLists;
    const auto addTriangle = [](AdjLists &adj, const Triangle &t)
    {
        adj[t.p1].emplace_back(t.p2), adj[t.p1].emplace_back(t.p3);
        adj[t.p2].emplace_back(t.p1), adj[t.p2].emplace_back(t.p3);
        adj[t.p3].emplace_back(t.p1), adj[t.p3].emplace_back(t.p2);
    };

    AdjLists thisAdj(totalNumPoints), otherAdj(totalNumPoints);
    for (int i = 0; i < numTriangles; ++i)
    {
        const Triangle &thisT = triangles[i];
        addTriangle(thisAdj, thisT);
        const Triangle &otherT = tri[i];
        addTriangle(otherAdj, otherT);
    }

    assert(totalNumPoints == thisAdj.size());
    assert(totalNumPoints == otherAdj.size());
    for (int i = 0; i < totalNumPoints; ++i)
    {
        std::sort(thisAdj[i].begin(), thisAdj[i].end()), std::unique(thisAdj[i].begin(), thisAdj[i].end());
        std::sort(otherAdj[i].begin(), otherAdj[i].end()), std::unique(otherAdj[i].begin(), otherAdj[i].end());
        if (thisAdj[i] != otherAdj[i])
        {
            TLOG(INFO) << "adjacency for vertex " << i << " is not the same";
            return false;
        }
    }

    return true;
}

/// Draw connected component containing edge "initEdge" into a given cv::Mat.
void Delaunay::DelaunayImpl::plotTriangulation(cv::Mat &img, EdgeIdx le, EdgeIdx re, EdgeIdx base)
{
    EdgeIdx initEdge = le;
    if (le == INVALID_EDGE)
    {
        if (re == INVALID_EDGE)
        {
            if (base == INVALID_EDGE)
                return;
            else
                initEdge = base;
        }
        else
            initEdge = re;
    }

    if (img.empty())
        img = cv::Mat::zeros(cv::Size(1800, 1000), CV_8UC3);
    else
        memset(img.data, 0, img.total() * img.elemSize());

    // calculate bounding box to get optimal scale
    short maxI = 0, maxJ = 0;
    for (int i = 0; i < totalNumPoints; ++i)
    {
        const PointIJ &p = P[i];
        maxI = std::max(maxI, p.i);
        maxJ = std::max(maxJ, p.j);
    }
    const float scaleI = 0.9f * (float(img.rows) / float(maxI));
    const float scaleJ = 0.9f * (float(img.cols) / float(maxJ));
    const int ofs = 5;  // margins

    std::queue<EdgeIdx> q;
    q.push(initEdge);
    std::vector<bool> added(totalNumPoints), visited(totalNumPoints);
    added[E[initEdge].origPnt] = true;

    int leftJ = 0xFFFF, rightJ = 0;

    while (!q.empty())
    {
        const EdgeIdx currEdgeIdx = q.front();
        q.pop();

        const uint16_t currPntIdx = E[currEdgeIdx].origPnt;
        const PointIJ &currPnt = P[currPntIdx];

        leftJ = std::min(leftJ, int(currPnt.j));
        rightJ = std::max(rightJ, int(currPnt.j));

        EdgeIdx edgeIdx = currEdgeIdx;
        do
        {
            const uint16_t destPntIdx = destPnt(edgeIdx);
            const PointIJ &destPnt = P[destPntIdx];

            auto color = cv::Scalar(0xFF, 0xFF, 0xFF);
            bool specialFace = false;
            int thickness = 1;
            if (edgeIdx == le)
                color = cv::Scalar(0, 0xFF, 0), specialFace = true, thickness = 3;
            else if (edgeIdx == re)
                color = cv::Scalar(0xFF, 0, 0), specialFace = true, thickness = 2;

            if (edgeIdx == base)
                color = cv::Scalar(0, 0xA5, 0xFF), specialFace = true, thickness = 5;

            if (!visited[destPntIdx] || specialFace)
            {
                cv::line(img,
                         cv::Point2i(int(currPnt.j * scaleJ + ofs), int(currPnt.i * scaleI + ofs)),
                         cv::Point2i(int(destPnt.j * scaleJ + ofs), int(destPnt.i * scaleI + ofs)),
                         color,
                         thickness);
            }

            if (specialFace)
                cv::circle(img, cv::Point2i(int(currPnt.j * scaleJ + ofs), int(currPnt.i * scaleI + ofs)), 4, color, 4);

            if (!added[destPntIdx])
            {
                q.push(E[edgeIdx].symEdge);
                added[destPntIdx] = true;
            }

            edgeIdx = E[edgeIdx].nextCcwEdge;
        } while (edgeIdx != currEdgeIdx);

        visited[currPntIdx] = true;
        assert(added[currPntIdx]);
    }

    if (rightJ * scaleJ - leftJ * scaleJ > 30)
    {
        for (int i = 0; i < totalNumPoints; ++i)
        {
            const PointIJ &p = P[i];
            cv::Vec3b &pixel = img.at<cv::Vec3b>(int(p.i * scaleI + ofs), int(p.j * scaleJ + ofs));
            pixel[0] = 0x77;
            pixel[1] = pixel[2] = 0x55;
        }
    }
}

void Delaunay::DelaunayImpl::showTriangulation(cv::Mat &img, EdgeIdx le, EdgeIdx re, EdgeIdx base)
{
    plotTriangulation(img, le, re, base);
    cv::imshow("tri", img);
    cv::waitKey();
}


// Wrapper class implementation.

/// Delaunay object is created once and can calculate many different triangulations.
/// No need to destroy and create it from scratch each time.
Delaunay::Delaunay()
{
    data.reset(new DelaunayImpl);
}

/// Need non-default dtor for "private implementation" with std::unique_ptr.
Delaunay::~Delaunay()
{
}

/// Little helper that does sorting, initialization and triangulation in a single call.
void Delaunay::operator()(std::vector<PointIJ> &points, std::vector<short> &indexMap)
{
    data->sortPoints(points, indexMap);
    data->init(points);
    data->triangulate();
}

void Delaunay::sortPoints(std::vector<PointIJ> &points, std::vector<short> &indexMap)
{
    data->sortPoints(points, indexMap);
}

void Delaunay::init(const std::vector<PointIJ> &points)
{
    data->init(points);
}

void Delaunay::triangulate()
{
    data->triangulate();
}

void Delaunay::generateTriangles()
{
    data->generateTriangles();
}

void Delaunay::plotTriangulation(cv::Mat &img)
{
    data->plotTriangulation(img, data->leftmostEdge);
}

void Delaunay::showTriangulation()
{
    cv::Mat image;
    data->showTriangulation(image, data->leftmostEdge);
}

void Delaunay::saveTriangulation(const std::string &filename, int numP, const PointIJ *p, int numT, const Triangle *t)
{
    DelaunayImpl::saveTriangulation(filename, numP, p, numT, t);
}

void Delaunay::loadTriangulation(const std::string &filename, std::vector<PointIJ> &p, std::vector<Triangle> &t)
{
    DelaunayImpl::loadTriangulation(filename, p, t);
}

bool Delaunay::isEqualTo(const std::vector<PointIJ> &p, const std::vector<Triangle> &t) const
{
    return data->isEqualTo(p, t);
}

void Delaunay::getTriangles(Triangle *&t, int &num)
{
    return data->getTriangles(t, num);
}
