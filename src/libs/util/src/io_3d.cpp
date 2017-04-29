#include <map>
#include <queue>
#include <vector>
#include <fstream>

#include <util/util.hpp>
#include <util/io_3d.hpp>
#include <util/geometry.hpp>
#include <util/tiny_logger.hpp>
#include <util/string_utils.hpp>


namespace
{

// local data structures

struct PropertyGroup
{
    PropertyGroup(const std::vector<std::string> &propertyNames, const std::string &type, char *dataPtr)
        : propertyNames(propertyNames)
        , type(type)
        , dataPtr(dataPtr)
    {
    }

    std::vector<std::string> propertyNames;
    std::string type;
    char *dataPtr = nullptr;
    int offset = 0;
    bool found = false;
};

struct Element
{
    int count;
    std::vector<std::string> propertyNames;
    std::vector<std::string> propertyTypes;
    std::vector<PropertyGroup> propertiesToRead;
};

// local helper functions
int typeToSize(const std::string &type)
{
    if (type == "float")
        return 4;
    else if (type == "uchar")
        return 1;
    else
        TLOG(ERROR) << "type " << type << " is unknown";

    return 0;
}

}


bool loadBinaryPly(const std::string &filename,
                   std::vector<cv::Point3f> *vertices,
                   std::vector<cv::Vec3b> *vertexColors)
{
    // It's very convenient to open file as binary, but read header as text using getline.
    // This may break one day, e.g. on a file with \r\n line endings.
    std::ifstream ply{ filename, std::ios::binary };
    std::string line;
    bool isOk;

    if (!std::getline(ply, line) || line != "ply")
    {
        TLOG(ERROR) << "expected 'ply' header line, format not supported";
        return false;
    }

    std::vector<std::string> tokens;
    if (!std::getline(ply, line) || (tokens = splitString(line, " ")).size() != 3)
    {
        TLOG(ERROR) << "ply format " << line << " cannot be interpreted (expected 3 tokens)";
        return false;
    }
    if (tokens.front() != "format")
    {
        TLOG(ERROR) << "ply format string should start with 'format' keyword, found: " << line;
        return false;
    }

    const std::string format{ tokens[1] }, version{ tokens[2] };
    if (format == "ascii" || format == "binary_big_endian")
    {
        TLOG(ERROR) << "format " << format << " " << version << " not supported. Only binary_little_endian is currently supported.";
        return false;
    }

    std::map<std::string, Element> elements;
    std::queue<std::string> elementOrder;

    while (std::getline(ply, line) && line != "end_header")
    {
        tokens = splitString(line, " ");

        if (tokens.size() == 3 && tokens.front() == "element")
        {
            Element element;
            const std::string elementName = tokens[1];
            element.count = stringTo<int>(tokens[2], isOk);
            if (!isOk)
            {
                TLOG(ERROR) << "could not parse the element count, token is: " << tokens[2];
                return false;
            }

            while (std::getline(ply, line) && startsWith(line, "property"))
            {
                tokens = splitString(line, " ");
                if (tokens.size() != 3)
                {
                    TLOG(ERROR) << "only parameters described by three tokens are supported, line is: " << line;
                    return false;
                }

                element.propertyTypes.emplace_back(tokens[1]);
                element.propertyNames.emplace_back(tokens[2]);
            }

            elements[elementName] = element;
            elementOrder.push(elementName);

            if (line == "end_header")
                break;
        }
        else if (!tokens.empty() && tokens.front() == "comment")
        {
            continue;
        }
        else
        {
            TLOG(WARNING) << "unknown header token: " << line;
            continue;
        }
    };

    // determine what data we actually need to read (and allocate buffers)
    const bool canReadVertices = elements.count("vertex") > 0,
               needVertices = vertices || vertexColors;

    if (needVertices && !canReadVertices)
        TLOG(ERROR) << "vertex element not found in .ply file";
    else
    {
        Element &vertexElem = elements["vertex"];
        if (vertices)
        {
            vertices->resize(vertexElem.count);
            vertexElem.propertiesToRead.push_back(PropertyGroup({"x", "y", "z"}, "float", (char *)vertices->data()));
        }
        if (vertexColors)
        {
            vertexColors->resize(vertexElem.count);
            vertexElem.propertiesToRead.push_back(PropertyGroup({ "red", "green", "blue" }, "uchar", (char *)vertexColors->data()));
        }
    }

    std::vector<char> elementData;

    while (!elementOrder.empty())
    {
        const std::string elementName{ elementOrder.front() };
        elementOrder.pop();
        Element &element = elements[elementName];
        int elementSize = 0;

        // learn which properties are actually in the data, also learn offset of each property group and total size of element
        assert(element.propertyNames.size() == element.propertyTypes.size());
        for (size_t i = 0; i < element.propertyNames.size(); ++i)
        {
            const std::string &type = element.propertyTypes[i];
            for (auto &group : element.propertiesToRead)
            {
                // check following sequence of properties matches property group that we want to read (e.g. x,y,z)
                if (group.found)
                    continue;
                if (group.type != type)
                    continue;
                if (element.propertyNames.size() - i < group.propertyNames.size())
                    continue;
                if (std::equal(group.propertyNames.begin(), group.propertyNames.end(), element.propertyNames.begin() + i))
                {
                    group.found = true;
                    group.offset = elementSize;
                    break;
                }
            }

            elementSize += typeToSize(type);
        }

        elementData.resize(element.count * elementSize);
        ply.read((char *)elementData.data(), elementData.size());

        for (const auto &group : element.propertiesToRead)
        {
            if (!group.found)
            {
                TLOG(ERROR) << "could not read property " << elementName << group.propertyNames;
                continue;
            }

            const size_t groupSize = group.propertyNames.size() * typeToSize(group.type);
            memcpyStride(group.dataPtr, (char *)elementData.data(), int(groupSize), element.count, group.offset, elementSize);
        }
    }

    return true;
}

std::string plyHeader(const std::vector<cv::Point3f> *vertices)
{
    std::ostringstream header;
    header << "ply\n";
    header << "format binary_little_endian 1.0\n";
    header << "comment Physical units: meters\n";

    if (vertices)
    {
        header << "element vertex " << vertices->size() << "\n";
        for (char c = 'x'; c <= 'z'; ++c)
            header << "property float " << c << "\n";
    }

    header << "end_header\n";
    return header.str();
}

/// Disclaimer: this is a very limited version of binary writer, made just for debugging.
bool saveBinaryPly(const std::string &filename, const std::vector<cv::Point3f> *vertices)
{
    std::ofstream ply{ filename, std::ios::binary };
    const auto header = plyHeader(vertices);
    ply.write(header.c_str(), header.length());

    if (ply && vertices)
        ply.write((const char *)vertices->data(), vertices->size() * sizeof(cv::Point3f));

    return bool(ply);
}
