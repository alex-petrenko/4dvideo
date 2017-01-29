#include <GLFW/glfw3.h>

#include <util/tiny_logger.hpp>


namespace
{

GLFWwindow *window = nullptr;

}


void drawLoop()
{
    while (true)
    {

    }
}

int main()
{
    if (glfwInit())
        TLOG(FATAL) << "Could not init glfw!";

    glfwWindowHint(GLFW_DEPTH_BITS, 16);
    glfwWindowHint(GLFW_SAMPLES, 4);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);  // to make MacOS happy; should not be needed
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

    const int screenWidth = 640, screenHeight = 360;
    window = glfwCreateWindow(screenWidth, screenHeight, "3d_video_player", NULL, NULL);

    if (window)
    {
        drawLoop();

        TLOG(INFO) << "App finished! Destroy window";
        glfwDestroyWindow(window);
    }
    else
    {
        TLOG(ERROR) << "Could not create window!";
    }

    return 0;
}
