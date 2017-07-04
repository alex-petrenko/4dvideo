#include <3dvideo/params.hpp>


Params & Params::instance()
{
    static Params params;
    return params;
}

Params::Params()
{
    // mesher params
    {
        auto &p = mesherP;
        p.sideLengthThreshold2D = 16;
        p.sideLengthThreshold3D = 0.08f;
        p.zThreshold = 0.05f;
    }

    // filter params
    {
        auto &p = filterP;
        p.minDepthMm = 500;
        p.maxDepthMm = 1100;
        p.purgeRadius = 3;
        p.curvatureThresholdMm = 12;
        p.minDepthClusterAreaCoeff = 0.001f;  // min 0.1% of the depth image area
    }
}
