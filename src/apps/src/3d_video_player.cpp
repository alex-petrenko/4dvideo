#include <mutex>
#include <chrono>
#include <thread>
#include <fstream>
#include <iomanip>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#ifdef _MSC_VER
    #pragma warning(push)
    #pragma warning(disable: 4310)  // cast truncates constant value
#endif
#include <glm/glm.hpp>
#include <glm/ext.hpp>
#ifdef _MSC_VER
    #pragma warning(pop)
#endif

#include <tri/triangulation.hpp>

#include <util/util.hpp>
#include <util/io_3d.hpp>
#include <util/geometry.hpp>
#include <util/test_utils.hpp>
#include <util/tiny_logger.hpp>
#include <util/opengl_utils.hpp>
#include <util/tiny_profiler.hpp>
#include <util/filesystem_utils.hpp>


namespace
{

// local variables

GLFWwindow *window = nullptr;
int viewportW = 0, viewportH = 0;

bool initialized = false;
bool isLongPress = false;

//std::vector<cv::Point3f> cloud;
std::vector<PointIJ> points;
cv::Point3f modelCenter;
Delaunay delaunay;
int numTriangles, num3DTriangles;
Triangle *triangles;

struct Triangle3D  // <-- temporary slow version, should replace with indexed mode
{
    cv::Point3f p1, p2, p3;

    cv::Point3f a() const { return p2 - p1; }
    cv::Point3f b() const { return p3 - p2; }
    cv::Point3f c() const { return p1 - p3; }
};
Triangle3D triangles3D[maxNumTriangles];
Triangle3D pointNormals[maxNumTriangles];  // TODO 

glm::mat4 scaleMatrix, rotation, translationMatrix;
glm::mat4 mvp;

GLint vertexID, uvID, colorID;
GLint transformUniformID;
GLint program;

ShaderLoader *shaderLoader;

const char *vertexShader =
"#version 330 core\n"
"layout(location = 0) in vec3 vertexPosition_modelspace;"
"layout(location = 1) in vec3 vertexNormal;"
""
"uniform mat4 transform;"
""
"out vec3 v;"
"out vec3 normal;"
""
"void main()"
"{"
//"    gl_Position = vec4((vertexPosition.y - 320.0) / 320.0, (180.0 - vertexPosition.x) / 180.0, 0, 1);"
//"    gl_PointSize = 13;"
"    gl_Position = transform * vec4(vertexPosition_modelspace, 1.0);"
"    v = vertexPosition_modelspace;"
"    normal = vertexNormal;"
//"    color = vertexColor;"
"}";

const char *fragmentShader =
"#version 330 core\n"
""
"in vec3 v;"
"in vec3 normal;"
""
"out vec4 color;"
""
"void main()"
"{"
"    vec3 lightPos = vec3(0, 0, 3);"
"    vec3 lightDirection = normalize(lightPos - v);"
"    color = vec4(0.3, 0.3, 0.3, 0.0) + vec4(vec3(0.5, 0.5, 0.5) * max(float(dot(normal, lightDirection)), 0.0), 1.0);"
//"    color = vec4(1,1,1,1);"
"}";



// functions

void glfwWindowKeyCallback(GLFWwindow *, int key, int scancode, int, int)
{
    TLOG(INFO) << "Key " << key << " " << scancode << " pressed";
}

void glfwScrollCallback(GLFWwindow *, double, double yScroll)
{
    double scale;
    if (yScroll > 0)
        scale = pow(1.1, std::abs(yScroll));
    else
        scale = pow(0.9, std::abs(yScroll));
    TLOG(INFO) << __FUNCTION__ << " scale is " << scale;
    scaleMatrix = glm::scale(scaleMatrix, glm::vec3(float(scale)));
    isLongPress = false;
}

void computeMatricesFromInputs()
{
    // glfwGetTime is called only once, the first time this function is called
    static double lastTime = glfwGetTime();
    static double longPressStarted = glfwGetTime();

    // Compute time difference between current and last frame
    double currentTime = glfwGetTime();
    float deltaTime = float(currentTime - lastTime);

    // Get mouse position
    double xpos, ypos;
    glfwGetCursorPos(window, &xpos, &ypos);

    static int prevCursorPositionX = int(xpos), prevCursorPositionY = int(ypos);

    if (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS)
    {
        float dx = float(xpos - prevCursorPositionX);
        float dy = float(ypos - prevCursorPositionY);
        //doSimpleRotation(glm::radians(dy / speedCoeff), glm::radians(dx / speedCoeff), 0.0f);

        float angleX = glm::radians(dy / 10);
        float angleY = glm::radians(-dx / 10);

        rotation = glm::rotate(rotation, angleX, glm::vec3(rotation[0][0], rotation[1][0], rotation[2][0]));
        rotation = glm::rotate(rotation, angleY, glm::vec3(rotation[0][1], rotation[1][1], rotation[2][1]));
        // z-rotation is not represented (can use touchscreen two-finger gesture for this)

        if (dx != 0 || dy != 0)
        {
            isLongPress = false;
        }
        else
        {
            if (!isLongPress)
            {
                isLongPress = true;
                longPressStarted = glfwGetTime();
            }

            const double longPressDuration = glfwGetTime() - longPressStarted;
            if (longPressDuration > 0.8f)
            {
                // reset position
                rotation = glm::mat4(1.0f);
                scaleMatrix = glm::mat4(1.0f);
                isLongPress = false;
            }
        }
    }
    else
    {
        isLongPress = false;
    }

    const float focus = 520.965f;
    const float cx = 319.223f, cy = 175.641f;
    CameraParams camera(focus, cx, cy, 640, 360);
    camera.scale(2);

    glm::mat4 projectionMatrix = projectionMatrixFromPinholeCamera(camera, 0.1f, 100.0f);

    glm::mat4 viewMatrix = glm::lookAt(
        glm::vec3(0.0f, 0.0f, 0.0f),  // Camera is here
        glm::vec3(0.0f, 0.0f, 1.0f),  // and looks at the origin
        glm::vec3(0.0f, -1.0f, 0.0f)  // head is up
    );

    glm::mat4 translateToOrigin = glm::translate(glm::mat4(1.0f), glm::vec3(-modelCenter.x, -modelCenter.y, -modelCenter.z));
    glm::mat4 translateBack = glm::translate(glm::mat4(1.0f), glm::vec3(modelCenter.x, modelCenter.y, modelCenter.z));

    glm::mat4 modelMatrix = translateBack * rotation * scaleMatrix * translateToOrigin;
    mvp = projectionMatrix * viewMatrix * modelMatrix;

    prevCursorPositionX = int(xpos), prevCursorPositionY = int(ypos);

    // For the next frame, the "last time" will be "now"
    lastTime = currentTime;
}

}


int currentFrameIdx = 0;
bool isPlayingBack = false, newFrame = true;
std::chrono::time_point<std::chrono::system_clock> playbackStarted;
std::mutex framesMutex;

struct Frame
{
    std::vector<cv::Point3f> frameCloud;
    double timestamp;
};
std::vector<Frame> frames;

void generateFrame()
{
    std::unique_lock<std::mutex> lock(framesMutex);

    if (!isPlayingBack)
    {
        assert(currentFrameIdx == 0);
        playbackStarted = std::chrono::system_clock::now();
        isPlayingBack = true;
    }
    else
    {
        if (currentFrameIdx + 1 < frames.size())
        {
            // play next frame if it's time
        }
        else
        {
            TLOG(INFO) << "reached the end of the dataset";
            currentFrameIdx = 0;
            isPlayingBack = false;
            return;
        }
    }

    const double frameTargetTime = (frames[currentFrameIdx + 1].timestamp - frames[0].timestamp) / 1.4;
    const auto passedTimeMs = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - playbackStarted).count();
    const double passedTime = double(passedTimeMs) / 1000.0;

    if (passedTime < frameTargetTime)
    {
        // too soon to play next frame
        return;
    }

    ++currentFrameIdx;
    newFrame = true;

    const Frame &frame = frames[currentFrameIdx];
    points.clear();

    const int w = 640, h = 360;
    const float f = 520.965f, cx = 319.223f, cy = 175.641f;  // parameters of Tango camera
    int iImg, jImg;
    ushort depth;
    for (size_t i = 0; i < frame.frameCloud.size(); ++i)
        if (project3dPointTo2d(frame.frameCloud[i], f, cx, cy, w, h, iImg, jImg, depth))
            points.emplace_back(iImg, jImg);
        else
            points.emplace_back(0, 0);

    std::vector<short> indexMap(points.size());
    delaunay(points, indexMap);
    delaunay.generateTriangles();
    delaunay.getTriangles(triangles, numTriangles);

    const float sideLengthThreshold = 0.1f;  // in meters
    const float zThreshold = 0.05f;

    int j = 0;
    for (int i = 0; i < numTriangles; ++i)
    {
        Triangle3D &t3d = triangles3D[j];
        const Triangle &t = triangles[i];

        t3d.p1 = frame.frameCloud[indexMap[t.p1]];
        t3d.p2 = frame.frameCloud[indexMap[t.p2]];
        t3d.p3 = frame.frameCloud[indexMap[t.p3]];

        float minZ = t3d.p1.z, maxZ = t3d.p1.z;
        minZ = std::min(minZ, t3d.p2.z);
        minZ = std::min(minZ, t3d.p3.z);
        maxZ = std::max(maxZ, t3d.p2.z);
        maxZ = std::max(maxZ, t3d.p3.z);
        if (maxZ - minZ > zThreshold)
            continue;

        const float a = float(cv::norm(t3d.a()));
        if (a > sideLengthThreshold) continue;
        const float b = float(cv::norm(t3d.b()));
        if (b > sideLengthThreshold) continue;
        const float c = float(cv::norm(t3d.c()));
        if (c > sideLengthThreshold) continue;

        const cv::Point3f n = triNormal(t3d.p1, t3d.p2, t3d.p3);
        // all three points have same normal TODO: optimize
        pointNormals[j].p1 = pointNormals[j].p2 = pointNormals[j].p3 = n;

        ++j;
    }
    num3DTriangles = j;

    static bool meanPointCalculated = false;
    if (!meanPointCalculated)
        modelCenter = meanPoint(frame.frameCloud), meanPointCalculated = true;
}


void drawTriangles()
{
    static GLuint vertexArrayID;
    static GLuint vertexBuffer;
    static GLuint normalBuffer;
    static GLuint triangleCenterBuffer;    

    if (!initialized)
    {
        shaderLoader = new ShaderLoader(vertexShader, fragmentShader, "test");
        program = shaderLoader->getProgram();

        glGenVertexArrays(1, &vertexArrayID);
        glBindVertexArray(vertexArrayID);

        glGenBuffers(1, &vertexBuffer);
        glBindBuffer(GL_ARRAY_BUFFER, vertexBuffer);
        glGenBuffers(1, &normalBuffer);
        glBindBuffer(GL_ARRAY_BUFFER, normalBuffer);

        transformUniformID = glGetUniformLocation(program, "transform");

        glClearColor(0.0f, 0.0f, 0.0f, 1);
        glEnable(GL_DEPTH_TEST);
        glDepthFunc(GL_LESS);

        initialized = true;
    }

    if (newFrame)
    {
        static int count = 0;
        glBindVertexArray(vertexArrayID);
        glBindBuffer(GL_ARRAY_BUFFER, vertexBuffer);
        glBufferData(GL_ARRAY_BUFFER, num3DTriangles * sizeof(triangles3D[0]), triangles3D, GL_DYNAMIC_DRAW);
        glBindBuffer(GL_ARRAY_BUFFER, normalBuffer);
        glBufferData(GL_ARRAY_BUFFER, num3DTriangles * sizeof(pointNormals[0]), pointNormals, GL_DYNAMIC_DRAW);
    }

    computeMatricesFromInputs();

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);


    glUseProgram(program);
    
    glEnableVertexAttribArray(0);
    glBindBuffer(GL_ARRAY_BUFFER, vertexBuffer);
    glVertexAttribPointer(
        0,                  // attribute 0. No particular reason for 0, but must match the layout in the shader.
        3,                  // size
        GL_FLOAT,           // type
        GL_FALSE,           // normalized?
        0,                  // stride
        (void*)0            // array buffer offset
    );

    glEnableVertexAttribArray(1);
    glBindBuffer(GL_ARRAY_BUFFER, normalBuffer);
    glVertexAttribPointer(
        1,
        3,                  // size
        GL_FLOAT,           // type
        GL_FALSE,           // normalized?
        0,                  // stride
        (void*)0            // array buffer offset
    );

    glUniformMatrix4fv(transformUniformID, 1, GL_FALSE, glm::value_ptr(mvp));

    glDrawArrays(GL_TRIANGLES, 0, 3 * numTriangles);

    glDisableVertexAttribArray(0);
    newFrame = false;
}


void drawLoop()
{
    glfwSetKeyCallback(window, glfwWindowKeyCallback);
    glfwSetScrollCallback(window, glfwScrollCallback);

    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    glfwGetFramebufferSize(window, &viewportW, &viewportH);

    while (!glfwWindowShouldClose(window))
    {
        generateFrame();

        drawTriangles();
        glfwSwapBuffers(window);
        glfwPollEvents();
    }
}

struct TangoPoint
{
    float x, y, z, c;
};

TangoPoint tangoPoints[100000];

float extrRotation[4];
float extrTranslation[3];


void readDataset(const std::string &datasetPath)
{
    //std::ifstream dataset(R"(C:\all\projects\itseez\data\testing\dataset2.bin)", std::ios::binary);
    std::ifstream dataset(R"(C:\all\projects\itseez\data\testing\special_datasets\002_yoga_wall.bin)", std::ios::binary);
    //std::ifstream dataset(R"(C:\all\projects\itseez\data\testing\special_datasets\003_push_ups.bin)", std::ios::binary);
    //std::ifstream dataset(R"(C:\all\projects\itseez\data\testing\1487499749_dataset.bin)", std::ios::binary);

    cv::Mat imageBgr(720, 1080, CV_8UC3);
    cv::Mat image(3 * 720 / 2, 1280, CV_8UC1);

    int numFrames = 0;
    while (dataset)
    {
        double timestamp;
        dataset.read((char *)&timestamp, sizeof(timestamp));
        endianSwap(&timestamp);
        dataset.read((char *)image.data, image.total());
        // cv::cvtColor(image, imageBgr, cv::COLOR_YUV2BGR_NV21);

        if (!dataset) break;
        double pointCloudTimestamp;
        dataset.read((char *)&pointCloudTimestamp, sizeof(pointCloudTimestamp));
        endianSwap(&pointCloudTimestamp);

        if (!dataset) break;
        int numPoints;
        dataset.read((char *)&numPoints, sizeof(numPoints));
        endianSwap(&numPoints);
        TLOG(INFO) << "point cloud timestamp: " << pointCloudTimestamp << " num points: " << numPoints << " size: " << numPoints * 4 * 4;

        if (numPoints > 0)
        {
            if (!dataset) break;
            int numPointBytes;
            dataset.read((char *)&numPointBytes, sizeof(numPointBytes));
            endianSwap(&numPointBytes);
            if (numPoints*4*4 != numPointBytes)
                TLOG(INFO) << "Num point bytes: " << numPointBytes;
            if (!dataset) break;
            dataset.read((char *)tangoPoints, numPointBytes);

            std::unique_lock<std::mutex> lock(framesMutex);
            frames.emplace_back();
            Frame &currentFrame = frames.back();
            currentFrame.timestamp = pointCloudTimestamp;
            currentFrame.frameCloud.resize(numPoints);
            for (int i = 0; i < numPoints; ++i)
            {
                endianSwap(&tangoPoints[i].x);
                endianSwap(&tangoPoints[i].y);
                endianSwap(&tangoPoints[i].z);
                currentFrame.frameCloud[i] = cv::Point3f(tangoPoints[i].x, tangoPoints[i].y, tangoPoints[i].z);
            }
        }

        if (!dataset) break;
        dataset.read((char *)extrRotation, sizeof(extrRotation));
        for (int i = 0; i < ARR_LENGTH(extrRotation); ++i)
            endianSwap(&extrRotation[i]);

        if (!dataset) break;
        dataset.read((char *)extrTranslation, sizeof(extrTranslation));
        for (int i = 0; i < ARR_LENGTH(extrTranslation); ++i)
            endianSwap(&extrTranslation[i]);

        //int w = 640, h = 360;
        //float f = 520.965f, cx = 319.223f, cy = 175.641f;  // parameters of Tango camera

        //int coeff = 2;
        //w *= coeff, h *= coeff;
        //f *= coeff, cx *= coeff, cy *= coeff;

        //uint16_t depth;
        //int iImg, jImg;
        //for (int i = 0; i < numPoints; ++i)
        //{
        //    const cv::Point3f p(tangoPoints[i].x, tangoPoints[i].y, tangoPoints[i].z);
        //    project3dPointTo2d(p, f, cx, cy, w, h, iImg, jImg, depth);

        //    cv::drawMarker(imageBgr, cv::Point(jImg, iImg), cv::Scalar(0xFF, 0xFF, 0xFF), cv::MARKER_SQUARE, 3);
        //}

        //if (numPoints > 0)
        //{
        //    cv::imshow("image", imageBgr);
        //    cv::waitKey();
        //}

        ++numFrames;
        TLOG(INFO) << "num frames: " << numFrames;
    }
}


int main(int argc, char *argv[])
{
    const int numArgs = 2;
    if (argc != numArgs)
        TLOG(FATAL) << "Expected " << numArgs << " arguments, got " << argc;

    int arg = 1;
    const std::string datasetPath(argv[arg++]);

    std::thread readingThread(readDataset, datasetPath);

    //const std::string testCloudFilename(pathJoin(getTestDataFolder(), "test_point_cloud.ply"));
    //const bool isOk = loadBinaryPly(testCloudFilename, &cloud);
    //TLOG(INFO) << "load binary ply status ok? " << isOk;

    //const int w = 640, h = 360;
    //const float f = 520.965f, cx = 319.223f, cy = 175.641f;  // parameters of Tango camera
    //int iImg, jImg;
    //ushort depth;
    //for (size_t i = 0; i < cloud.size(); ++i)
    //    if (project3dPointTo2d(cloud[i], f, cx, cy, w, h, iImg, jImg, depth))
    //        points.emplace_back(iImg, jImg);

    //std::vector<short> indexMap(points.size());
    //delaunay(points, indexMap);
    //delaunay.generateTriangles();
    //delaunay.getTriangles(triangles, numTriangles);

    //for (int i = 0; i < numTriangles; ++i)
    //{
    //    Triangle3D &t3d = triangles3D[i];
    //    const Triangle &t = triangles[i];

    //    t3d.p1 = cloud[indexMap[t.p1]];
    //    t3d.p2 = cloud[indexMap[t.p2]];
    //    t3d.p3 = cloud[indexMap[t.p3]];

    //    cv::Point3f n = triNormal(t3d.p1, t3d.p2, t3d.p3);
    //    // all three points have same normal TODO: optimize
    //    pointNormals[i].p1 = pointNormals[i].p2 = pointNormals[i].p3 = n;
    //}

    //modelCenter = meanPoint(cloud);

    if (!glfwInit())
        TLOG(FATAL) << "Could not init glfw!";

    glfwWindowHint(GLFW_DEPTH_BITS, 16);
    glfwWindowHint(GLFW_SAMPLES, 4);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);  // to make MacOS happy; should not be needed
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

    const int screenWidth = 640 * 2, screenHeight = 360 * 2;
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

    readingThread.join();

    return EXIT_SUCCESS;
}
