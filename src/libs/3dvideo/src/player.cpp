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
"layout(location = 2) in vec2 vertexUV;"
""
"uniform mat4 transform;"
""
"out vec3 v;"
"out vec3 normal;"
"out vec2 uv;"
""
"void main()"
"{"
"    gl_Position = transform * vec4(vertexPosition_modelspace, 1.0);"
"    v = vertexPosition_modelspace;"
"    normal = vertexNormal;"
"    uv = vertexUV;"
"}";

const char *fragmentShader =
"#version 330 core\n"
"in vec3 v;"
"in vec3 normal;"
"in vec2 uv;"
""
"uniform sampler2D textureSampler;"
""
"out vec3 color;"
""
"void main()"
"{"
"    vec3 lightPos = vec3(0, 0, 3);"
"    vec3 lightDirection = normalize(lightPos - v);"
// "    color = vec4(0.3, 0.3, 0.3, 0.0) + vec4(vec3(0.5, 0.5, 0.5) * max(float(dot(normal, lightDirection)), 0.0), 1.0);"
"    color = texture(textureSampler, uv).rgb;"
"}";

}


class Player::PlayerImpl
{
    friend class Player;

public:
    PlayerImpl(Player &parent)
        : parent(parent)
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
        float scale = float(targetScreenWidth) / depthCam.w;
        depthCam.scale(scale);
        screenW = depthCam.w, screenH = depthCam.h;

        if (!glfwInit())
            TLOG(FATAL) << "Could not init glfw!";

        glfwWindowHint(GLFW_DEPTH_BITS, 16);
        glfwWindowHint(GLFW_SAMPLES, 4);
        glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
        glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
        glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);  // to make MacOS happy; should not be needed
        glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
        
        TLOG(INFO) << "Creating window...";
        window = glfwCreateWindow(screenW, screenH, "4D player", nullptr, nullptr);
        TLOG(INFO) << "Window created!";

        glfwSetKeyCallback(window, glfwWindowKeyCallback);
        glfwSetScrollCallback(window, glfwScrollCallback);

        glfwMakeContextCurrent(window);
        glfwSwapInterval(1);

        initOpenGL();
    }

    void initOpenGL()
    {
        TLOG(INFO);

        shaderLoader = std::make_shared<ShaderLoader>(vertexShader, fragmentShader, "4D");
        program = shaderLoader->getProgram();

        glGenVertexArrays(1, &vertexArrayID);
        glBindVertexArray(vertexArrayID);

        glGenBuffers(1, &vertexBuffer);
        //glBindBuffer(GL_ARRAY_BUFFER, vertexBuffer);
        glGenBuffers(1, &normalBuffer);
        //glBindBuffer(GL_ARRAY_BUFFER, normalBuffer);
        glGenBuffers(1, &uvBuffer);
        //glBindBuffer(GL_ARRAY_BUFFER, uvBuffer);
        glGenBuffers(1, &indexBuffer);

        transformUniformID = glGetUniformLocation(program, "transform");

        glGenTextures(1, &texture);
        glBindTexture(GL_TEXTURE_2D, texture);

        // should not actually be needed, just in case; default is GL_REPEAT
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
        // trilinear filtering
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);

        glClearColor(0.0f, 0.0f, 0.0f, 1);
        glEnable(GL_DEPTH_TEST);
        glDepthFunc(GL_LESS);
    }

    void onScroll(double scroll)
    {
        double scaleCoeff;
        if (scroll > 0)
            scaleCoeff = pow(1.1, std::abs(scroll));
        else
            scaleCoeff = pow(0.9, std::abs(scroll));
        scaleMatrix = glm::scale(scaleMatrix, glm::vec3(float(scaleCoeff)));
        isLongPress = false;
    }

    bool loopBody()
    {
        auto &queue = parent.q;

        if (!currentFrame)
            queue.pop(currentFrame, 10);

        if (currentFrame)
        {
            if (currentFrame->frame2D->frameNumber < lastPlayedFrame)
            {
                // beginning of dataset!
                playbackStarted = std::chrono::system_clock::now();
                firstFrameTimestamp = currentFrame->frame2D->dTimestamp;
            }

            if (canPlayCurrentFrame())
            {
                lastPlayedFrame = currentFrame->frame2D->frameNumber;
                frameToDraw = currentFrame;

                if (!meanPointCalculated)
                    modelCenter = meanPoint(currentFrame->cloud), meanPointCalculated = true;

                currentFrame.reset();
            }
        }

        draw();

        glfwSwapBuffers(window);
        glfwPollEvents();
        return !glfwWindowShouldClose(window);
    }

    bool canPlayCurrentFrame()
    {
        if (!currentFrame)
            return false;

        const auto targetPlaybackTimeUs = double(currentFrame->frame2D->dTimestamp - firstFrameTimestamp);
        const auto passedTimeUs = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now() - playbackStarted).count();
        return passedTimeUs >= targetPlaybackTimeUs;
    }

    void computeMatricesFromInputs()
    {
        // glfwGetTime is called only once, the first time this function is called
        static double lastTime = glfwGetTime();
        static double longPressStarted = glfwGetTime();

        // get mouse position
        double xpos, ypos;
        glfwGetCursorPos(window, &xpos, &ypos);

        static int prevCursorPositionX = int(xpos), prevCursorPositionY = int(ypos);

        if (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS)
        {
            const float dx = float(xpos - prevCursorPositionX);
            const float dy = float(ypos - prevCursorPositionY);
            const float angleX = glm::radians(dy / 10);
            const float angleY = glm::radians(-dx / 10);

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

        const CameraParams camera = depthCam;
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
    }

    void draw()
    {
        if (frameToDraw)
        {
            indexedMode = frameToDraw->indexedMode;

            auto &frame2D = frameToDraw->frame2D;
            glBindVertexArray(vertexArrayID);

            if (indexedMode)
            {
                numElements = GLsizei(frameToDraw->triangles.size());

                glBindBuffer(GL_ARRAY_BUFFER, vertexBuffer);
                glBufferData(GL_ARRAY_BUFFER, frameToDraw->cloud.size() * sizeof(cv::Point3f), frameToDraw->cloud.data(), GL_DYNAMIC_DRAW);
                glBindBuffer(GL_ARRAY_BUFFER, normalBuffer);
                glBufferData(GL_ARRAY_BUFFER, frameToDraw->normals.size() * sizeof(cv::Point3f), frameToDraw->normals.data(), GL_DYNAMIC_DRAW);
                glBindBuffer(GL_ARRAY_BUFFER, uvBuffer);
                glBufferData(GL_ARRAY_BUFFER, frameToDraw->uv.size() * sizeof(cv::Point2f), frameToDraw->uv.data(), GL_DYNAMIC_DRAW);
                glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, indexBuffer);
                glBufferData(GL_ELEMENT_ARRAY_BUFFER, frameToDraw->triangles.size() * sizeof(Triangle), frameToDraw->triangles.data(), GL_DYNAMIC_DRAW);
            }
            else
            {
                numElements = GLsizei(frameToDraw->num3DTriangles);

                glBindBuffer(GL_ARRAY_BUFFER, vertexBuffer);
                glBufferData(GL_ARRAY_BUFFER, numElements * sizeof(Triangle3D), frameToDraw->triangles3D.data(), GL_DYNAMIC_DRAW);
                glBindBuffer(GL_ARRAY_BUFFER, normalBuffer);
                glBufferData(GL_ARRAY_BUFFER, numElements * sizeof(Triangle3D), frameToDraw->trianglesNormals.data(), GL_DYNAMIC_DRAW);
                glBindBuffer(GL_ARRAY_BUFFER, uvBuffer);
                glBufferData(GL_ARRAY_BUFFER, numElements * sizeof(TriangleUV), frameToDraw->trianglesUv.data(), GL_DYNAMIC_DRAW);
            }

            // loading texture data to GPU
            glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, frame2D->color.cols, frame2D->color.rows, 0, GL_BGR, GL_UNSIGNED_BYTE, frame2D->color.data);
            glGenerateMipmap(GL_TEXTURE_2D);

            frameToDraw.reset();
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

        glEnableVertexAttribArray(2);
        glBindBuffer(GL_ARRAY_BUFFER, uvBuffer);
        glVertexAttribPointer(
            2,
            2,                  // size (2 floats for uv)
            GL_FLOAT,           // type
            GL_FALSE,           // normalized?
            0,                  // stride
            (void*)0            // array buffer offset
        );

        if (indexedMode)
            glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, indexBuffer);

        glUniformMatrix4fv(transformUniformID, 1, GL_FALSE, glm::value_ptr(mvp));

        if (indexedMode)
            glDrawElements(GL_TRIANGLES, 3 * numElements, GL_UNSIGNED_SHORT, 0);
        else
            glDrawArrays(GL_TRIANGLES, 0, 3 * numElements);

        glDisableVertexAttribArray(0);
    }

private:
    Player &parent;

    // GLFW

    GLFWwindow *window = nullptr;

    // input

    bool isLongPress = false;

    // OpenGL stuff

    std::shared_ptr<ShaderLoader> shaderLoader;
    GLint program;
    GLuint vertexArrayID;
    GLuint vertexBuffer, normalBuffer, uvBuffer, indexBuffer;
    GLint transformUniformID;
    GLuint texture;

    glm::mat4 scaleMatrix, rotation, translationMatrix;
    glm::mat4 mvp;

    bool meanPointCalculated = false;
    cv::Point3f modelCenter;

    bool indexedMode = false;
    GLsizei numElements = 0;

    // player state

    int lastPlayedFrame = std::numeric_limits<int>::max();
    int64_t firstFrameTimestamp = 0;
    std::chrono::time_point<std::chrono::system_clock> playbackStarted;

    std::shared_ptr<MeshFrame> currentFrame, frameToDraw;
    
    // camera and screen

    CameraParams depthCam;
    int screenW = 0, screenH = 0;
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


Player::Player(MeshFrameQueue &q, CancellationToken &cancellationToken)
    : MeshFrameConsumer(q, cancellationToken)
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
