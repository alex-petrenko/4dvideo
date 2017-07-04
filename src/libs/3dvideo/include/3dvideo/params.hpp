#pragma once

#include <stdint.h>


class Params
{
public:
    struct MesherParams
    {
        /// Max length of the side of any triangle in pixels when projected onto depth camera plane.
        /// Helps to avoid thin triangles on the boundaries of objects.
        float sideLengthThreshold2D;

        /// Max length of the side of any 3D triangle in meters.
        float sideLengthThreshold3D;

        /// Max difference between minimum and maximum Z-coordinate of any 3D triangle. In meters.
        float zThreshold;
    };

    struct FilterParams
    {
        /// Filter depth points that are closer.
        uint16_t minDepthMm;

        /// Filter depth points that are further away.
        uint16_t maxDepthMm;

        /// Eliminate depth points within this radius around the current depth point.
        int purgeRadius;

        /// Don't thin out depth in high-curvature areas, where z-difference beween adjacent points is higher than this threshold.
        uint16_t curvatureThresholdMm;

        /// Minimum area of depth cluster, in parts of the depth cam resolution (e.g. 0.01 = 1% of depth image area).
        float minDepthClusterAreaCoeff;
    };

public:
    static Params & instance();

    const MesherParams & getMesherParams() const { return mesherP; }
    const FilterParams & getFilterParams() const { return filterP; }

private:
    Params();
    Params(const Params &) = delete;
    void operator=(const Params &) = delete;

private:
    MesherParams mesherP;
    FilterParams filterP;
};

inline const Params & algoParams() { return Params::instance(); }
inline const Params::MesherParams & mesherParams() { return algoParams().getMesherParams(); }
inline const Params::FilterParams & filterParams() { return algoParams().getFilterParams(); }
