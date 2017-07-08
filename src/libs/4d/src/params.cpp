#include <4d/params.hpp>


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
        p.triSideLengthThreshold2D = 16;
        p.triSideLengthThreshold3D = 0.08f;
        p.zThreshold = 0.05f;
    }

    // filter params
    {
        auto &p = filterP;
        p.minDepthMm = 100;
        p.maxDepthMm = 2600;
        p.purgeRadius = 3;
        p.curvatureThresholdMm = 12;
        p.minDepthClusterAreaCoeff = 0.001f;  // min 0.1% of the depth image area
    }

    // animation params
    {
        auto &p = animP;
        p.textureScale = 0.5f;
        p.batchSize = 8;
    }

    constexpr bool needCustomParams = false;
    if (needCustomParams)
        SetCustomParams();
}

void Params::SetCustomParams()
{
}
