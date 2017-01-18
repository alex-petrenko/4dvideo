#pragma once

#include <vector>

#include <opencv2/core.hpp>


/// Read ply file. Return true on success.
bool loadBinaryPly(const std::string &filename,
                   std::vector<cv::Point3f> *vertices = nullptr,
                   std::vector<cv::Vec3b> *vertexColors = nullptr);
