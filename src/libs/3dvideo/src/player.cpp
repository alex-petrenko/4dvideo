#ifdef _MSC_VER
    #pragma warning(push)
    #pragma warning(disable: 4310)  // cast truncates constant value
#endif
#include <glm/glm.hpp>
#include <glm/ext.hpp>
#ifdef _MSC_VER
    #pragma warning(pop)
#endif

#include <util/tiny_logger.hpp>
#include <util/opengl_utils.hpp>

#include <tri/triangulation.hpp>

#include <3dvideo/player.hpp>
#include <3dvideo/app_state.hpp>


using namespace std::chrono_literals;


namespace
{

// pointer to active Player object for GLFW callbacks
Player::PlayerImpl *activePlayer = nullptr;

// GLFW callbacks
void glfwWindowKeyCallback(GLFWwindow *, int key, int scancode, int, int);
void glfwScrollCallback(GLFWwindow *, double, double yScroll);

// OpenGL stuff

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


struct Triangle3D  // <-- temporary slow version, should replace with indexed mode
{
    cv::Point3f p1, p2, p3;

    cv::Point3f a() const { return p2 - p1; }
    cv::Point3f b() const { return p3 - p2; }
    cv::Point3f c() const { return p1 - p3; }
};

}


class Player::PlayerImpl
{
    friend class Player;

public:
    PlayerImpl(Player &parent)
        : parent(parent)
        , triangles3D(maxNumTriangles)
        , pointNormals(maxNumTriangles)
    {
    }

    ~PlayerImpl()
    {
        if (window)
            glfwDestroyWindow(window);
    }

    void init()
    {
        const SensorManager &sensorManager = appState().getSensorManager();
        DepthDataFormat depthFormat;
        sensorManager.getDepthParams(depthCam, depthFormat);
        screenW = depthCam.w;
        screenH = depthCam.h;

        if (!glfwInit())
            TLOG(FATAL) << "Could not init glfw!";

        glfwWindowHint(GLFW_DEPTH_BITS, 16);
        glfwWindowHint(GLFW_SAMPLES, 4);
        glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
        glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
        glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);  // to make MacOS happy; should not be needed
        glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

        window = glfwCreateWindow(screenW, screenH, "4D player", nullptr, nullptr);

        glfwSetKeyCallback(window, glfwWindowKeyCallback);
        glfwSetScrollCallback(window, glfwScrollCallback);

        glfwMakeContextCurrent(window);
        glfwSwapInterval(1);

        initOpenGL();
    }

    void initOpenGL()
    {
        shaderLoader = std::make_shared<ShaderLoader>(vertexShader, fragmentShader, "4D");
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
    }

    void onScroll(double scroll)
    {
    /*    double scale;
        if (yScroll > 0)
        scale = pow(1.1, std::abs(yScroll));
        else
        scale = pow(0.9, std::abs(yScroll));
        TLOG(INFO) << __FUNCTION__ << " scale is " << scale;
        scaleMatrix = glm::scale(scaleMatrix, glm::vec3(float(scale)));
        isLongPress = false;*/
    }

    bool loopBody()
    {
        FrameQueue &queue = parent.q;

        if (!currentFrame)
            queue.pop(currentFrame, 10);

        if (canPlayCurrentFrame())
            setupNewFrame();

        draw();

        glfwSwapBuffers(window);
        glfwPollEvents();
        return !glfwWindowShouldClose(window);
    }

    bool canPlayCurrentFrame()
    {
        if (!currentFrame)
            return false;

        static int counter = 0;
        ++counter;
        if (counter % 2 ==0)
            return true;
        else return false;

        return true;
    }

    void setupNewFrame()
    {
        cloud.clear(), points.clear();

        const auto depth = currentFrame->depth;
        const uint16_t minDepth = 200, maxDepth = 6000;
        for (int i = 0; i < depth.rows; i += 2)
            for (int j = 0; j < depth.cols; j += 2)
            {
                const uint16_t d = depth.at<uint16_t>(i, j);
                if (d > minDepth && d < maxDepth && points.size() < std::numeric_limits<short>::max())
                {
                    points.emplace_back(i, j);
                    cloud.emplace_back(project2dPointTo3d(i, j, d, depthCam));
                }
            }

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

            t3d.p1 = cloud[indexMap[t.p1]];
            t3d.p2 = cloud[indexMap[t.p2]];
            t3d.p3 = cloud[indexMap[t.p3]];

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

        if (!meanPointCalculated)
            modelCenter = meanPoint(cloud), meanPointCalculated = true;
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

        /*const float focus = 520.965f;
        const float cx = 319.223f, cy = 175.641f;
        CameraParams camera(focus, cx, cy, 640, 360);
        camera.scale(2);*/

        CameraParams camera = depthCam;
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

    void draw()
    {
        if (currentFrame)
        {
            glBindVertexArray(vertexArrayID);
            glBindBuffer(GL_ARRAY_BUFFER, vertexBuffer);
            glBufferData(GL_ARRAY_BUFFER, num3DTriangles * sizeof(Triangle3D), triangles3D.data(), GL_DYNAMIC_DRAW);
            glBindBuffer(GL_ARRAY_BUFFER, normalBuffer);
            glBufferData(GL_ARRAY_BUFFER, num3DTriangles * sizeof(Triangle3D), pointNormals.data(), GL_DYNAMIC_DRAW);
            currentFrame.reset();
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
    }

private:
    Player &parent;

    // GLFW

    GLFWwindow *window = nullptr;
    int screenW = 0, screenH = 0;

    // input

    bool isLongPress = false;

    // OpenGL stuff

    std::shared_ptr<ShaderLoader> shaderLoader;
    GLint program;
    GLuint vertexArrayID, vertexBuffer, normalBuffer;
    GLint transformUniformID;
    /*GLint vertexID, uvID, colorID;*/

    glm::mat4 scaleMatrix, rotation, translationMatrix;
    glm::mat4 mvp;

    // player state

    std::shared_ptr<Frame> currentFrame;
    std::vector<PointIJ> points;
    std::vector<cv::Point3f> cloud;
    std::vector<Triangle3D> triangles3D, pointNormals;

    Triangle *triangles = nullptr;
    int numTriangles = 0, num3DTriangles = 0;

    bool meanPointCalculated = false;
    cv::Point3f modelCenter;

    CameraParams depthCam;
    Delaunay delaunay;
};


namespace
{

// GLFW callbacks (implementation)

void glfwWindowKeyCallback(GLFWwindow *, int key, int scancode, int, int)
{
    TLOG(INFO) << "Key " << key << " " << scancode << " pressed";
}

void glfwScrollCallback(GLFWwindow *, double, double yScroll)
{
    if (activePlayer)
        activePlayer->onScroll(yScroll);
}

}


Player::Player(FrameQueue &q, CancellationToken &cancellationToken)
    : FrameConsumer(q, cancellationToken)
{
    data = std::make_unique<PlayerImpl>(*this);
    activePlayer = data.get();
}

Player::~Player()
{
    activePlayer = nullptr;
}

void Player::init()
{
    TLOG(INFO);
    const SensorManager &sensorManager = appState().getSensorManager();
    while (!cancel && !sensorManager.isInitialized())
        std::this_thread::sleep_for(30ms);
    data->init();
}

void Player::run()
{
    bool shouldContinue = true;
    while (shouldContinue)
    {
        shouldContinue = data->loopBody();
        shouldContinue &= !cancel;
    }
}
