#pragma once

#include <string>

#include <GL/glew.h>

#include <GLFW/glfw3.h>

#include <util/camera.hpp>


class ShaderLoader
{
public:
    ShaderLoader(const std::string &vShaderCode, const std::string &fShaderCode, const std::string &name = std::string());
    ~ShaderLoader();

    GLint getProgram() const;

private:
    GLuint vShader, fShader;
    GLint program;

    std::string name;
};


glm::mat4 projectionMatrixFromPinholeCamera(const CameraParams &camera, float near, float far);
