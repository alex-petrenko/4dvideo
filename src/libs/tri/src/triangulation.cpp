#include <queue>
#include <cassert>

#include <opencv2/imgproc/imgproc.hpp>

#include <tri/triangulation.hpp>

#include <util/geometry.hpp>


// this structure is left unpacked, cause it's faster
struct TriEdge
{
    uint16_t origPnt;  // index of edge's origin point
    EdgeIdx symEdge;  // index of pair edge, with same endpoints and opposite direction
    EdgeIdx nextCcwEdge;  // next counterclockwise (CCW) edge around the origin
    EdgeIdx prevCcwEdge;  // previous CCW edge around the origin (or next CW edge)

    FORCE_INLINE TriEdge * sym() const;
    FORCE_INLINE uint16_t destPnt() const;
};

struct Delaunay
{
    const PointIJ *pointsData;  // use this raw buffer to avoid calling stl methods
    int totalNumPoints;

    int numEdges;
    TriEdge edges[maxNumEdges];
};

Delaunay data;


#define E(idx) (&data.edges[idx])
#define P(idx) (data.pointsData + idx)

// Edge methods.

FORCE_INLINE TriEdge * TriEdge::sym() const
{
    return E(symEdge);
}

/// Returns index of edge destination point.
FORCE_INLINE uint16_t TriEdge::destPnt() const
{
    return sym()->origPnt;
}


// Generic helpers.

#define DBG_SHOW_TRIANG 0
cv::Mat triangImg;

/// Draw connected component containing edge "initialEdgeIdx".
void showTriangulation(cv::Mat &img, EdgeIdx le, EdgeIdx re, EdgeIdx base)
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
    for (int i = 0; i < data.totalNumPoints; ++i)
    {
        const PointIJ * const p = data.pointsData + i;
        maxI = std::max(maxI, p->i);
        maxJ = std::max(maxJ, p->j);
    }
    const float scaleI = 0.9f * (float(img.rows) / float(maxI));
    const float scaleJ = 0.9f * (float(img.cols) / float(maxJ));
    const int ofs = 5;  // margins

    std::queue<EdgeIdx> q;
    q.push(initEdge);
    std::vector<bool> added(data.totalNumPoints), visited(data.totalNumPoints);
    added[E(initEdge)->origPnt] = true;

    int leftJ = 0xFFFF, rightJ = 0;

    while (!q.empty())
    {
        const EdgeIdx currEdgeIdx = q.front();
        q.pop();

        const uint16_t currPntIdx = E(currEdgeIdx)->origPnt;
        const PointIJ * const currPnt = P(currPntIdx);

        leftJ = std::min(leftJ, int(currPnt->j));
        rightJ = std::max(rightJ, int(currPnt->j));

        EdgeIdx edgeIdx = currEdgeIdx;
        do
        {
            const uint16_t destPntIdx = E(edgeIdx)->destPnt();
            const PointIJ * const destPnt = P(destPntIdx);

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
                cv::line(img,
                         cv::Point2i(int(currPnt->j * scaleJ + ofs), int(currPnt->i * scaleI + ofs)),
                         cv::Point2i(int(destPnt->j * scaleJ + ofs), int(destPnt->i * scaleI + ofs)),
                         color,
                         thickness);

            if (specialFace)
                cv::circle(img, cv::Point2i(int(currPnt->j * scaleJ + ofs), int(currPnt->i * scaleI + ofs)), 4, color, 4);

            if (!added[destPntIdx])
            {
                q.push(E(edgeIdx)->symEdge);
                added[destPntIdx] = true;
            }

            edgeIdx = E(edgeIdx)->nextCcwEdge;
        } while (edgeIdx != currEdgeIdx);

        visited[currPntIdx] = true;
        assert(added[currPntIdx]);
    }

    if (rightJ * scaleJ - leftJ * scaleJ > 30)
    {
        for (int i = 0; i < data.totalNumPoints; ++i)
        {
            const PointIJ * const p = data.pointsData + i;
            cv::Vec3b &pixel = img.at<cv::Vec3b>(int(p->i * scaleI + ofs), int(p->j * scaleJ + ofs));
            pixel[0] = 0x77;
            pixel[1] = pixel[2] = 0x55;
        }

#if DBG_SHOW_TRIANG
        cv::imshow("triang", img);
        cv::waitKey();
#endif
    }
}


// Geometry helpers.

/// p1,p2,p3 should be oriented counterclockwise!
FORCE_INLINE bool inCircle(uint16_t p1, uint16_t p2, uint16_t p3, uint16_t p)
{
    return inCircle(P(p1)->j, P(p1)->i, P(p2)->j, P(p2)->i, P(p3)->j, P(p3)->i, P(p)->j, P(p)->i);
}

FORCE_INLINE Orientation orientationPointEdge(uint16_t p, TriEdge *e)
{
    const uint16_t orig = e->origPnt, dest = e->destPnt();
    return triOrientation(P(orig)->j, P(orig)->i, P(dest)->j, P(dest)->i, P(p)->j, P(p)->i);
}

FORCE_INLINE bool isRightOf(uint16_t p, EdgeIdx e)
{
    return orientationPointEdge(p, E(e)) == ORIENT_CW;
}

FORCE_INLINE bool isLeftOf(uint16_t p, EdgeIdx e)
{
    return orientationPointEdge(p, E(e)) == ORIENT_CCW;
}

// Topology helpers.

/// Creates pair of oriented edges orig->dest and dest->orig.
FORCE_INLINE EdgeIdx makeEdge(uint16_t orig, uint16_t dest)
{
    const EdgeIdx e1Idx = data.numEdges, e2Idx = e1Idx + 1;
    data.numEdges += 2;
    assert(data.numEdges < maxNumEdges / 2);

    TriEdge * const e1 = E(e1Idx);
    TriEdge * const e2 = E(e2Idx);

    e1->origPnt = orig;
    e2->origPnt = dest;

    e1->symEdge = e2Idx;
    e2->symEdge = e1Idx;

    e1->nextCcwEdge = e1->prevCcwEdge = e1Idx;
    e2->nextCcwEdge = e2->prevCcwEdge = e2Idx;

    return e1Idx;
}

/// Attach newEdge to the origin of oldEdge immediately after oldEdge counterclockwise.
FORCE_INLINE void join(EdgeIdx newEdgeIdx, EdgeIdx oldEdgeIdx)
{
    TriEdge * const newEdge = E(newEdgeIdx);
    TriEdge * const oldEdge = E(oldEdgeIdx);

    assert(newEdge->origPnt == oldEdge->origPnt);

    newEdge->nextCcwEdge = oldEdge->nextCcwEdge;
    oldEdge->nextCcwEdge = newEdgeIdx;

    E(newEdge->nextCcwEdge)->prevCcwEdge = newEdgeIdx;
    newEdge->prevCcwEdge = oldEdgeIdx;
}

/// Connects destination of a to origin of b. Returns e = a.dest->b.orig.
FORCE_INLINE EdgeIdx connect(EdgeIdx aIdx, EdgeIdx bIdx)
{
    TriEdge * const a = E(aIdx);
    TriEdge * const b = E(bIdx);

    const EdgeIdx cIdx = makeEdge(a->destPnt(), b->origPnt);
    TriEdge * const c = E(cIdx);
    join(cIdx, a->sym()->prevCcwEdge);
    join(c->symEdge, bIdx);

    assert(a->destPnt() == c->origPnt);
    assert(b->origPnt == c->destPnt());
    assert(a->destPnt() == c->sym()->destPnt());
    assert(b->origPnt == c->sym()->origPnt);
    return cIdx;
}

/// Removes references to given edge from the linked list around it's origin.
FORCE_INLINE void unlink(EdgeIdx eIdx)
{
    TriEdge * const e = E(eIdx);
    E(e->nextCcwEdge)->prevCcwEdge = e->prevCcwEdge;
    E(e->prevCcwEdge)->nextCcwEdge = e->nextCcwEdge;
}

/// Removes given edge and it's pair from the subdivision.
FORCE_INLINE void deleteEdge(EdgeIdx eIdx)
{
    unlink(eIdx);
    unlink(E(eIdx)->symEdge);
}

FORCE_INLINE void mergeTriangulations(EdgeIdx lle, EdgeIdx lre, EdgeIdx rle, EdgeIdx rre, EdgeIdx &le, EdgeIdx &re)
{
    /*
    * Merge phase. Joining left and right triangulations into one.
    */

    // first, find the new base edge, the lower common tangent of left and right subdivisions.
    while (true)
    {
        // move edges down across convex hull of subdivision until we find a tangent
        if (isLeftOf(E(rle)->origPnt, lre))
        {
            // best right candidate for the tangent endpoint still "below", move lre further down
            lre = E(lre)->sym()->prevCcwEdge;
        }
        else if (isRightOf(E(lre)->origPnt, rle))
        {
            // best left candidate for the tangent endpoint still "below", move rle further down
            rle = E(rle)->sym()->nextCcwEdge;
        }
        else
        {
            // tangent endpoints are found
            break;
        }
    }

    // create "base" edge, we will work up from it merging the triangulations
    EdgeIdx base = connect(E(rle)->symEdge, lre);
    assert(!isLeftOf(E(lle)->origPnt, base));
    assert(!isLeftOf(E(rre)->origPnt, base));

    // it may be required to update le and re
    if (E(lle)->origPnt == E(base)->destPnt())
        lle = E(base)->symEdge;
    if (E(rre)->origPnt == E(base)->origPnt)
        rre = base;

    le = lle, re = rre;

    // Main merging code. Find left and right candidate and update base edge to move up.
    // When no candidates are left we reached the convex hull.
    // For more information read: http://www.sccg.sk/~samuelcik/dgs/quad_edge.pdf page 114.
    while (true)
    {
        EdgeIdx lCand = E(base)->sym()->nextCcwEdge;
        bool lCandFound = false;
        while (isRightOf(E(lCand)->destPnt(), base))
        {
            const EdgeIdx nextLCand = E(lCand)->nextCcwEdge;
            if (inCircle(E(base)->destPnt(), E(base)->origPnt, E(lCand)->destPnt(), E(nextLCand)->destPnt()))
            {
                assert(isRightOf(E(nextLCand)->destPnt(), base));
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

        //EdgeIdx lCand = E(base)->sym()->nextCcwEdge;
        //bool lCandFound = false;
        //if (isRightOf(E(lCand)->destPnt(), base))
        //{
        //    EdgeIdx nextLCand = E(lCand)->nextCcwEdge;
        //    // showTriangulation(triangImg, lCand, INVALID_EDGE, base);
        //    while (inCircle(E(base)->destPnt(), E(base)->origPnt, E(lCand)->destPnt(), E(nextLCand)->destPnt()))
        //    {
        //        assert(isRightOf(E(nextLCand)->destPnt(), base));
        //        deleteEdge(lCand);
        //        lCand = nextLCand;
        //        nextLCand = E(lCand)->nextCcwEdge;
        //        // showTriangulation(triangImg, lCand, INVALID_EDGE, base);
        //    }
        //    assert(isRightOf(E(lCand)->destPnt(), base));
        //    // left candidate is found, we can move on
        //    lCandFound = true;
        //}

        EdgeIdx rCand = E(base)->prevCcwEdge;
        bool rCandFound = false;
        while (isRightOf(E(rCand)->destPnt(), base))
        {
            const EdgeIdx nextRCand = E(rCand)->prevCcwEdge;
            // showTriangulation(triangImg, INVALID_EDGE, rCand, base);
            if (inCircle(E(base)->destPnt(), E(base)->origPnt, E(rCand)->destPnt(), E(nextRCand)->destPnt()))
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

        //EdgeIdx rCand = E(base)->prevCcwEdge;
        //bool rCandFound = false;
        //if (isRightOf(E(rCand)->destPnt(), base))
        //{
        //    EdgeIdx nextRCand = E(rCand)->prevCcwEdge;
        //    // showTriangulation(triangImg, INVALID_EDGE, rCand, base);
        //    while (inCircle(E(base)->destPnt(), E(base)->origPnt, E(rCand)->destPnt(), E(nextRCand)->destPnt()))
        //    {
        //        assert(isRightOf(E(nextRCand)->destPnt(), base));
        //        deleteEdge(rCand);
        //        rCand = nextRCand;
        //        nextRCand = E(rCand)->prevCcwEdge;
        //        // showTriangulation(triangImg, INVALID_EDGE, rCand, base);
        //    }
        //    assert(isRightOf(E(rCand)->destPnt(), base));
        //    // right candidate is found, we can move on
        //    rCandFound = true;
        //}

        if (!lCandFound && !rCandFound)
        {
            // cannot continue updating base edge, this should mean that we've reached upper common tangent of two subdivisions
            {
                // all triangulation should now be to the left of the base edge, let's do a couple of checks just in case
                assert(!isRightOf(E(lle)->origPnt, base));
                assert(!isRightOf(E(lre)->origPnt, base));
                assert(!isRightOf(E(rle)->origPnt, base));
                assert(!isRightOf(E(rre)->origPnt, base));
            }

            break;
        }
        else  // at least one valid candidate is found
        {
            if (!lCandFound || (rCandFound && inCircle(E(base)->destPnt(), E(base)->origPnt, E(lCand)->destPnt(), E(rCand)->destPnt())))
            {
                // either lCand not found or rCand destination is in lCand triangle circumcircle --> select rCand
                base = connect(rCand, E(base)->symEdge);
            }
            else
            {
                // select lCand
                base = connect(E(base)->symEdge, E(lCand)->symEdge);
            }
        }
    }
}

/// Main divide and conquer algorithm.
/// Each call modifies the subdivision structure and returns two edges from the convex hull.
/// le - index of CCW convex hull edge from leftmost vertex
/// re - index of CW convex hull edge from rightmost vertex
void triangulateSubset(uint16_t lIdx, uint16_t numPoints, EdgeIdx &le, EdgeIdx &re)
{
    assert(numPoints < maxNumPoints);

    if (numPoints == 2)
    {
        const uint16_t s1 = lIdx, s2 = s1 + 1;
        le = makeEdge(s1, s2);
        re = E(le)->symEdge;
    }
    else if (numPoints == 3)
    {
        const uint16_t s1 = lIdx, s2 = s1 + 1, s3 = s2 + 1;
        const EdgeIdx aIdx = makeEdge(s1, s2);
        const EdgeIdx bIdx = makeEdge(s2, s3);

        join(E(aIdx)->symEdge, bIdx);  // now a.sym and b are adjacent

        const Orientation pos = triOrientation(P(s1)->j, P(s1)->i, P(s2)->j, P(s2)->i, P(s3)->j, P(s3)->i);
        switch (pos)
        {
        case ORIENT_CCW:
            connect(bIdx, aIdx);
            le = aIdx;
            re = E(bIdx)->symEdge;
            break;
        case ORIENT_CW:
            re = connect(bIdx, aIdx);
            le = E(re)->symEdge;
            break;
        default:
            // points are collinear
            le = aIdx;
            re = E(bIdx)->symEdge;
            break;
        }
    }
    else
    {
        assert(numPoints >= 4);

        const int numRight = numPoints / 2, numLeft = numPoints - numRight;
        EdgeIdx lle;  // index of CCW convex hull edge from leftmost vertex of left triangulation
        EdgeIdx lre;  // index of CW convex hull edge from rightmost vertex of left triangulation
        EdgeIdx rle;  // index of CCW convex hull edge from leftmost vertex of right triangulation
        EdgeIdx rre;  // index of CW convex hull edge from rightmost vertex of right triangulation
        triangulateSubset(lIdx + numLeft, numRight, rle, rre);
        triangulateSubset(lIdx, numLeft, lle, lre);

        mergeTriangulations(lle, lre, rle, rre, le, re);
    }
}

#define USE_NON_RECURSIVE_TRIANGULATION 0
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
                *re = E(*le)->symEdge;
                --stackIdx;
            }
            else if (numPoints == 3)
            {
                const uint16_t s1 = lIdx, s2 = s1 + 1, s3 = s2 + 1;
                const EdgeIdx aIdx = makeEdge(s1, s2);
                const EdgeIdx bIdx = makeEdge(s2, s3);

                join(E(aIdx)->symEdge, bIdx);  // now a.sym and b are adjacent

                const Orientation pos = triOrientation(P(s1)->j, P(s1)->i, P(s2)->j, P(s2)->i, P(s3)->j, P(s3)->i);
                switch (pos)
                {
                case ORIENT_CCW:
                    connect(bIdx, aIdx);
                    *le = aIdx;
                    *re = E(bIdx)->symEdge;
                    break;
                case ORIENT_CW:
                    *re = connect(bIdx, aIdx);
                    *le = E(*re)->symEdge;
                    break;
                default:
                    // points are collinear
                    *le = aIdx;
                    *re = E(bIdx)->symEdge;
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


PointIJ pointsSortedByI[maxNumPoints];
short originalIdx[maxNumPoints];
short numPointsPerCoord[maxCoord], coordIdx[maxCoord];

// index map contains original index for every point in sorted sequence
void sortPoints(std::vector<PointIJ> &points, std::vector<short> &indexMap)
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

// accepts a sorted array of points without duplicates
void triangulate(std::vector<PointIJ> &points, EdgeIdx &le)
{
    assert(sizeof(TriEdge) <= 16);
    if (points.size() < 2)
        return;

    data.pointsData = points.data();
    data.totalNumPoints = int(points.size());
    data.numEdges = 0;

    EdgeIdx re;
    triangulateSubset(0, data.totalNumPoints, le, re);

#if DBG_SHOW_TRIANG
    showTriangulation(triangImg, le);
#endif
}

struct Triangle
{
    uint16_t p1, p2, p3;
};

EdgeIdx bfsQueue[maxNumEdges];
bool bfsVisited[maxNumEdges];

int numTriangles = 0;
Triangle triangles[maxNumTriangles];

void generateTriangles(EdgeIdx startEdge)
{
    memset(bfsVisited, 0, data.numEdges);
    numTriangles = 0;

    // triangulation is a single connected component, so we can just traverse
    // the whole structure collecting triangles along the way
    int queueHead = 0, queueTail = 0;
    bfsQueue[queueTail++] = startEdge;
    bfsVisited[startEdge] = true;

    while (queueTail != queueHead)
    {
        const EdgeIdx currEdgeIdx = bfsQueue[queueHead++];
        const TriEdge * const currEdge = E(currEdgeIdx);
        const EdgeIdx symEdgeIdx = currEdge->symEdge;

        // Try to find triangle to the right hand of the edge.
        const EdgeIdx side1 = E(currEdge->prevCcwEdge)->symEdge;
        const EdgeIdx side2 = E(symEdgeIdx)->nextCcwEdge;

        assert(E(side1)->destPnt() == currEdge->origPnt);
        assert(E(side2)->origPnt == currEdge->destPnt());

        if (E(side1)->origPnt == E(side2)->destPnt())
        {
            // Three edges indeed form an oriented clockwise triangle.
            // Let's add it to the list of triangles! (TODO: triangle strips?)
            Triangle &t = triangles[numTriangles++];
            t.p1 = currEdge->origPnt;
            t.p2 = E(side1)->origPnt;
            t.p3 = E(side2)->origPnt;

            // Add symmetric edges to the queue, they may belong to adjacent triangles.
            const EdgeIdx side1Sym = E(side1)->symEdge;
            const EdgeIdx side2Sym = E(side2)->symEdge;
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

    /*LOG(INFO) << __FUNCTION__ << "triangles generated:" << numTriangles;
    LOG(INFO) << __FUNCTION__ << "total queue size:" << queueTail;*/
}