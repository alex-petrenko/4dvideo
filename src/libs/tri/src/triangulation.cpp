#include <queue>
#include <cassert>
#include <fstream>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <tri/triangulation.hpp>

#include <util/util.hpp>
#include <util/geometry.hpp>
#include <util/tiny_logger.hpp>
#include <util/tiny_profiler.hpp>


namespace
{

// Visualization helpers. All visualization functionality must be disabled when performance is the goal.
#define WITH_VIS 1

enum class VisTag
{
    SUBDIVISION,
    FIND_CANDIDATES,
    UPDATE_BASE_EDGE,
    MERGE_COMPLETED,
    FINAL,
};

#if WITH_VIS
#define VIS(...) visualizeAll(__VA_ARGS__);
#define VIS_NFRAMES(n, ...) for (int visI = 0; visI < (n); ++visI) VIS(__VA_ARGS__)
#else
#define VIS(...)
#define VIS_NFRAMES(n, ...)
#endif

}


class Delaunay::DelaunayImpl
{
    friend class Delaunay;

private:
    DelaunayImpl()
        : totalNumPoints(0)
        , numEdges(0)
    {
        memset(E, INVALID_EDGE, sizeof(E));
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
        // Safety margin, might want to allocate bigger buffer if that's exceeded.
        // Can also add "defragmentation" for edge buffer to reduce memory usage.
        assert(numEdges < maxNumEdges * 2 / 3);  

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
        TriEdge &e = E[eIdx];
        E[e.nextCcwEdge].prevCcwEdge = e.prevCcwEdge;
        E[e.prevCcwEdge].nextCcwEdge = e.nextCcwEdge;
    }

    /// Removes given edge and its pair from the subdivision.
    FORCE_INLINE void deleteEdge(EdgeIdx eIdx)
    {
        unlink(eIdx);
        unlink(E[eIdx].symEdge);

        sym(eIdx).symEdge = INVALID_EDGE;
        E[eIdx].symEdge = INVALID_EDGE;
    }


    // Main algorithm.

    /// Call before each run of an algorithm.
    void init(const std::vector<PointIJ> &points);

    /// Fast radix sort.
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

    /// Draw the entire triangulation in a given cv::Mat.
    /// If starting edge is given, draw only the connected component containing this edge.
    /// Highlights leftmost, rightmost and base edges if they are given.
    void plotTriangulation(cv::Mat &img, EdgeIdx start = INVALID_EDGE, EdgeIdx le = INVALID_EDGE, EdgeIdx re = INVALID_EDGE, EdgeIdx base = INVALID_EDGE, EdgeIdx deleted = INVALID_EDGE);

    /// Show triangulation on a screen (mostly for debugging purposes).
    void showTriangulation(cv::Mat &img, EdgeIdx le, EdgeIdx re = INVALID_EDGE, EdgeIdx base = INVALID_EDGE);

    /// Set function to be called when intermediate state of the calculation is visualized.
    void setVisualizationCallback(const std::function<VisualizationCallback> &callback);

    /// Draw intermediate triangulation state into a cv::Mat and call visualization callback if one is set.
    void visualize(VisTag tag, bool clean = true, EdgeIdx start = INVALID_EDGE, EdgeIdx le = INVALID_EDGE, EdgeIdx re = INVALID_EDGE, EdgeIdx base = INVALID_EDGE, EdgeIdx deleted = INVALID_EDGE);

    /// Same as previous but with less boilerplate.
    void visualizeAll(VisTag tag, EdgeIdx le = INVALID_EDGE, EdgeIdx re = INVALID_EDGE, EdgeIdx base = INVALID_EDGE, EdgeIdx deleted = INVALID_EDGE);

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

    uint32_t numEdges = 0;
    TriEdge E[maxNumEdges];
    EdgeIdx leftmostEdge = INVALID_EDGE, rightmostEdge = INVALID_EDGE;  // valid only when triangulation is calculated

    uint32_t numTriangles = 0;
    Triangle triangles[maxNumTriangles];

    // auxiliary data for generateTriangles
    EdgeIdx bfsQueue[maxNumEdges];
    bool bfsVisited[maxNumEdges];

    // auxiliary data for radix sort
    PointIJ pointsSortedByI[maxNumPoints];
    short originalIdx[maxNumPoints];
    short numPointsPerCoord[maxCoord], coordIdx[maxCoord];

    // visualization
    std::function<VisualizationCallback> visualizationCallback;
    cv::Mat triImg;
    float visScaleI = 1, visScaleJ = 1;
    int visOfsI = 0, visOfsJ = 0;
    static constexpr int visW = 1920, visH = 1080;
    static constexpr bool visFixedResolution = true, visPreserveAspect = true;
};


/// Accepts a sorted array of points without duplicates.
void Delaunay::DelaunayImpl::init(const std::vector<PointIJ> &points)
{
    assert(sizeof(TriEdge) <= 16);
    P = points.data();
    assert(points.size() < std::numeric_limits<uint16_t>::max());
    totalNumPoints = uint16_t(points.size());
    numEdges = numTriangles = 0;

#if WITH_VIS
    triImg = cv::Mat();  // clean the image at each reinitialization
#endif
}

/// Algorithm only works with sorted points, so this function is used to sort them. Also removes duplicates from the sequence.
/// Returned index map contains original index for every point in sorted sequence.
/// This function is so messy because it's optimized for speed. It is several times faster than a combination of
/// standard library's std::sort and std::unique.
void Delaunay::DelaunayImpl::sortPoints(std::vector<PointIJ> &points, std::vector<short> &indexMap)
{
    assert(points.size() < std::numeric_limits<short>::max() - 1);
    assert(indexMap.size() == points.size());

    // Doing something like radix sort (1st - counting sort by i coord, then counting sort by j coord).
    // This is about 10x faster than std::sort.
    memset(numPointsPerCoord, 0, sizeof(numPointsPerCoord));
    memset(coordIdx, 0, sizeof(coordIdx));
    for (size_t i = 0; i < points.size(); ++i)
    {
        assert(points[i].i < maxCoord);
        ++numPointsPerCoord[points[i].i];
    }
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
    VIS_NFRAMES(2, VisTag::FINAL);
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
        VIS(VisTag::SUBDIVISION);
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
        VIS(VisTag::SUBDIVISION);
    }
    else
    {
        assert(numPoints >= 4);

        const uint16_t numRight = numPoints / 2, numLeft = numPoints - numRight;
        EdgeIdx lle;  // CCW convex hull edge starting at the leftmost vertex of left triangulation
        EdgeIdx lre;  // CW convex hull edge starting at the rightmost vertex of left triangulation
        EdgeIdx rle;  // CCW convex hull edge starting at the leftmost vertex of right triangulation
        EdgeIdx rre;  // CW convex hull edge starting at the rightmost vertex of right triangulation
        triangulateSubset(lIdx + numLeft, numRight, rle, rre);
        triangulateSubset(lIdx, numLeft, lle, lre);
        mergeTriangulations(lle, lre, rle, rre, le, re);
    }
}

/// Merge phase.
FORCE_INLINE void Delaunay::DelaunayImpl::mergeTriangulations(EdgeIdx lle, EdgeIdx lre, EdgeIdx rle, EdgeIdx rre, EdgeIdx &le, EdgeIdx &re)
{
    // first, find the new base edge, the lower common tangent of left and right subdivisions
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
    VIS(VisTag::UPDATE_BASE_EDGE, INVALID_EDGE, INVALID_EDGE, base);
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
        VIS(VisTag::FIND_CANDIDATES, lCand, INVALID_EDGE, base);
        bool lCandFound = false;
        while (isRightOf(destPnt(lCand), base))
        {
            const EdgeIdx nextLCand = E[lCand].nextCcwEdge;
            if (inCircle(destPnt(base), E[base].origPnt, destPnt(lCand), destPnt(nextLCand)))
            {
                assert(isRightOf(destPnt(nextLCand), base));
                VIS_NFRAMES(2, VisTag::FIND_CANDIDATES, INVALID_EDGE, INVALID_EDGE, base, lCand);
                deleteEdge(lCand);
                lCand = nextLCand;
                VIS(VisTag::FIND_CANDIDATES, lCand, INVALID_EDGE, base);
            }
            else
            {
                // left candidate is found, we can move on
                lCandFound = true;
                break;
            }
        }

        EdgeIdx rCand = E[base].prevCcwEdge;
        VIS(VisTag::FIND_CANDIDATES, lCand, rCand, base);
        bool rCandFound = false;
        while (isRightOf(destPnt(rCand), base))
        {
            const EdgeIdx nextRCand = E[rCand].prevCcwEdge;
            if (inCircle(destPnt(base), E[base].origPnt, destPnt(rCand), destPnt(nextRCand)))
            {
                VIS_NFRAMES(2, VisTag::FIND_CANDIDATES, lCand, INVALID_EDGE, base, rCand);
                deleteEdge(rCand);
                rCand = nextRCand;
                VIS(VisTag::FIND_CANDIDATES, lCand, rCand, base);
            }
            else
            {
                // right candidate is found, we can move on
                rCandFound = true;
                break;
            }
        }

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

            VIS(VisTag::MERGE_COMPLETED, INVALID_EDGE, INVALID_EDGE, base);
            break;
        }
        else  // at least one valid candidate is found
        {
            if (!lCandFound || (rCandFound && inCircle(destPnt(base), E[base].origPnt, destPnt(lCand), destPnt(rCand))))
            {
                // either lCand not found or rCand destination is in lCand triangle circumcircle --> select rCand
                VIS(VisTag::UPDATE_BASE_EDGE, INVALID_EDGE, rCand, base);
                base = connect(rCand, E[base].symEdge);
            }
            else
            {
                // select lCand
                VIS(VisTag::UPDATE_BASE_EDGE, lCand, INVALID_EDGE, base);
                base = connect(E[base].symEdge, E[lCand].symEdge);
            }

            VIS(VisTag::UPDATE_BASE_EDGE, INVALID_EDGE, INVALID_EDGE, base);
        }
    }
}

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
            // Let's add it to the list of triangles!
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
    for (size_t i = 0; i < numTriangles; ++i)
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
void Delaunay::DelaunayImpl::plotTriangulation(cv::Mat &img, EdgeIdx start, EdgeIdx le, EdgeIdx re, EdgeIdx base, EdgeIdx deleted)
{
    tprof().startTimer("plot");

    if (img.empty())
        img = cv::Mat::zeros(cv::Size(visW, visH), CV_8UC3);

    std::queue<EdgeIdx> q;
    std::vector<bool> addedPoints(totalNumPoints), visitedPoints(totalNumPoints), visitedEdges(numEdges);

    std::vector<EdgeIdx> edges;
    if (start == INVALID_EDGE)
    {
        // no start edge given, draw all graph components
        for (EdgeIdx e = 0; e < numEdges; ++e)
            if (E[e].symEdge != INVALID_EDGE)
                edges.push_back(e);
    }
    else
        edges.push_back(start);

    for (EdgeIdx e : edges)
    {
        if (visitedEdges[e])
            continue;

        q.push(e);
        visitedEdges[e] = visitedEdges[E[e].symEdge] = true;
        addedPoints[E[e].origPnt] = true;

        int iter = 0;
        while (!q.empty())
        {
            ++iter;
            const EdgeIdx currEdgeIdx = q.front();
            q.pop();

            const uint16_t currPntIdx = E[currEdgeIdx].origPnt;
            const PointIJ &currPnt = P[currPntIdx];

            EdgeIdx edgeIdx = currEdgeIdx;
            do
            {
                const uint16_t destPntIdx = destPnt(edgeIdx);
                const PointIJ &destPnt = P[destPntIdx];

                auto color = cv::Scalar(0x99, 0x99, 0x99);
                bool specialFace = false;
                int thickness = 1;
                if (edgeIdx == le)
                    color = cv::Scalar(0, 0xFF, 0), specialFace = true, ++thickness;
                else if (edgeIdx == re)
                    color = cv::Scalar(0xFF, 0, 0), specialFace = true, ++thickness;
                else if (edgeIdx == deleted)
                    color = cv::Scalar(0, 0, 0xFF), specialFace = true, thickness += 3;

                if (edgeIdx == base)
                    color = cv::Scalar(0, 0xA5, 0xFF), specialFace = true, ++thickness;

                if (!visitedPoints[destPntIdx] || specialFace)
                {
                    cv::line(img,
                             cv::Point2i(int(currPnt.j * visScaleJ + visOfsJ), int(currPnt.i * visScaleI + visOfsI)),
                             cv::Point2i(int(destPnt.j * visScaleJ + visOfsJ), int(destPnt.i * visScaleI + visOfsI)),
                             color,
                             thickness);
                }

                if (specialFace)
                    cv::circle(img, cv::Point2i(int(currPnt.j * visScaleJ + visOfsJ), int(currPnt.i * visScaleI + visOfsI)), 3, color, 3);

                if (!addedPoints[destPntIdx])
                {
                    q.push(E[edgeIdx].symEdge);
                    visitedEdges[edgeIdx] = visitedEdges[E[edgeIdx].symEdge] = true;
                    addedPoints[destPntIdx] = true;
                }

                edgeIdx = E[edgeIdx].nextCcwEdge;
            } while (edgeIdx != currEdgeIdx);

            visitedPoints[currPntIdx] = true;
            assert(addedPoints[currPntIdx]);
        }
    }

    const auto pclr = cv::Scalar_<uchar>(0x99, 0x99, 0x99);
    for (int i = 0; i < totalNumPoints; ++i)
    {
        const PointIJ &p = P[i];
        const cv::Point2i center{ int(p.j * visScaleJ + visOfsJ), int(p.i * visScaleI + visOfsI) };
        constexpr bool drawCircle = true;
        if (drawCircle)
            cv::circle(img, center, 1, pclr, 2);
        else
            img.at<cv::Vec3b>(center) = cv::Vec3b(pclr[0], pclr[1], pclr[2]);
    }

    tprof().stopTimer("plot");
}

void Delaunay::DelaunayImpl::showTriangulation(cv::Mat &img, EdgeIdx le, EdgeIdx re, EdgeIdx base)
{
    memset(img.data, 0, img.total() * img.elemSize());
    plotTriangulation(img, le, le, re, base);
    cv::imshow("tri", img);
    cv::waitKey();
}

void Delaunay::DelaunayImpl::setVisualizationCallback(const std::function<VisualizationCallback> &callback)
{
    visualizationCallback = callback;
}

void Delaunay::DelaunayImpl::visualize(VisTag tag, bool clean, EdgeIdx start, EdgeIdx le, EdgeIdx re, EdgeIdx base, EdgeIdx deleted)
{
    std::vector<VisTag> tags{ VisTag::SUBDIVISION };
    constexpr bool showAllStages = true;
    if (!contains(tags, tag) && !showAllStages)
        return;

    // initialize the visualization (determine the scale and resolution)
    if (triImg.empty())
    {
        // calculate bounding box to get optimal scale
        short maxI = 0, maxJ = 0;
        for (int i = 0; i < totalNumPoints; ++i)
        {
            const PointIJ &p = P[i];
            maxI = std::max(maxI, p.i);
            maxJ = std::max(maxJ, p.j);
        }

        constexpr float imgPercentage = 0.85f;
        if (visFixedResolution)
        {
            triImg = cv::Mat::zeros(cv::Size(visW, visH), CV_8UC3);
            visScaleI = imgPercentage * (float(triImg.rows) / float(maxI));
            visScaleJ = imgPercentage * (float(triImg.cols) / float(maxJ));
            visScaleI = visScaleJ = std::min(visScaleI, visScaleJ);
        }
        else
        {
            const int w = int(maxJ / imgPercentage), h = int(maxI / imgPercentage);
            triImg = cv::Mat::zeros(cv::Size(w, h), CV_8UC3);
            visScaleI = visScaleJ = 1;
        }

        // margins
        visOfsI = int((triImg.rows - maxI * visScaleI) / 2);
        visOfsJ = int((triImg.cols - maxJ * visScaleJ) / 2);
    }
    else if (clean)
        memset(triImg.data, 0, triImg.total() * triImg.elemSize());

    plotTriangulation(triImg, start, le, re, base, deleted);
    if (visualizationCallback)
        visualizationCallback(triImg);
}

void Delaunay::DelaunayImpl::visualizeAll(VisTag tag, EdgeIdx le, EdgeIdx re, EdgeIdx base, EdgeIdx deleted)
{
    visualize(tag, true, INVALID_EDGE, le, re, base, deleted);
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

void Delaunay::setVisualizationCallback(const std::function<VisualizationCallback> &callback)
{
    data->setVisualizationCallback(callback);
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
