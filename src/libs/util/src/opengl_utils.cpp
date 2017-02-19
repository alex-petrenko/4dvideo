#include <glm/glm.hpp>

#include <util/tiny_logger.hpp>
#include <util/opengl_utils.hpp>


namespace
{

GLuint loadShader(int kind, const std::string &code, const std::string &shaderName)
{
    GLuint shader = glCreateShader(kind);
    TLOG_IF(FATAL, !shader) << "could not create shader of kind " << kind << " shader name: " << shaderName;

    const char *codeStr = code.c_str();
    glShaderSource(shader, 1, &codeStr, nullptr);
    glCompileShader(shader);

    GLint compiled;
    glGetShaderiv(shader, GL_COMPILE_STATUS, &compiled);
    if (!compiled)
    {
        GLint logLength;
        glGetShaderiv(shader, GL_INFO_LOG_LENGTH, &logLength);
        if (logLength > 0)
        {
            std::vector<char> errorMsg(logLength + 1, '\0');
            glGetShaderInfoLog(shader, logLength, nullptr, errorMsg.data());
            TLOG(ERROR) << "compilation error msg: " << errorMsg.data();
        }

        glDeleteShader(shader);
        TLOG(FATAL) << "could not compile shader: " << shaderName;
    }

    return shader;
}

}


ShaderLoader::ShaderLoader(const std::string &vShaderCode, const std::string &fShaderCode, const std::string &name)
    : program(0)
    , name(name)
{
    TLOG_IF(FATAL, glewInit() != GLEW_OK) << "could not initialize GLEW";

    vShader = loadShader(GL_VERTEX_SHADER, vShaderCode, name);
    fShader = loadShader(GL_FRAGMENT_SHADER, fShaderCode, name);
    program = glCreateProgram();
    TLOG_IF(FATAL, !program) << "could not create program for shader" << name;

    glAttachShader(program, vShader), glAttachShader(program, fShader);
    glLinkProgram(program);

    GLint linked;
    glGetProgramiv(program, GL_LINK_STATUS, &linked);
    if (!linked)
    {
        GLint logLength;
        glGetProgramiv(program, GL_INFO_LOG_LENGTH, &logLength);
        if (logLength > 0)
        {
            std::vector<char> errorMsg(logLength + 1, '\0');
            glGetProgramInfoLog(program, logLength, nullptr, errorMsg.data());
            TLOG(ERROR) << "link error msg: " << errorMsg.data();
        }

        glDeleteProgram(program);
        TLOG(FATAL) << "could not link program";
    }
}

ShaderLoader::~ShaderLoader()
{
    if (!program)
        return;

    TLOG(INFO) << "dtor called for shader: " << name;
    glDetachShader(program, vShader), glDetachShader(program, fShader);
    glDeleteShader(vShader), glDeleteShader(fShader);
    glDeleteProgram(program);
}

GLint ShaderLoader::getProgram() const
{
    return program;
}

// http://www.songho.ca/opengl/gl_projectionmatrix.html
glm::mat4 projectionMatrixFromPinholeCamera(const CameraParams &c, float near, float far)
{
    return glm::mat4(
        2 * c.f / c.w, 0, 0, 0,
        0, 2 * c.f / c.h, 0, 0,
        -2 * c.cx / c.w + 1, 2 * c.cy / c.h - 1, -(far + near) / (far - near), -1,
        0, 0, -2.0 * far * near / (far - near), 0
    );
}
