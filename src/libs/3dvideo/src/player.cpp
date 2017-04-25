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

        return true;
    }

    void setupNewFrame()
    {
        cloud.clear(), points.clear();

        const auto depth = currentFrame->depth;
        const uint16_t minDepth = 200, maxDepth = 6000;
        for (int i = 0; i < depth.rows; ++i)
            for (int j = 0; j < depth.cols; ++j)
            {
                const uint16_t d = depth.at<uint16_t>(i, j);
                if (d > minDepth && d < maxDepth)
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
    }

    void draw()
    {

    }

private:
    Player &parent;

    // GLFW

    GLFWwindow *window = nullptr;
    int screenW = 0, screenH = 0;

    // OpenGL stuff

    std::shared_ptr<ShaderLoader> shaderLoader;
    GLint program;
    GLuint vertexArrayID, vertexBuffer, normalBuffer;
    GLint transformUniformID;
    /*GLint vertexID, uvID, colorID;*/

    // player state

    std::shared_ptr<Frame> currentFrame;
    std::vector<PointIJ> points;
    std::vector<cv::Point3f> cloud;
    std::vector<Triangle3D> triangles3D, pointNormals;

    Triangle *triangles = nullptr;
    int numTriangles = 0;

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
