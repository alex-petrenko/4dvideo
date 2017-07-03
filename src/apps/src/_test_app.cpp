#include <chrono>
#include <thread>
#include <iomanip>

#include <opencv2/highgui.hpp>

#include <util/io_3d.hpp>
#include <util/tiny_logger.hpp>
#include <util/filesystem_utils.hpp>

#include <3dvideo/dataset_reader.hpp>
#include <3dvideo/dataset_writer.hpp>
#include <3dvideo/data_visualizer.hpp>


bool replace(std::string& str, const std::string& from, const std::string& to) {
    size_t start_pos = str.find(from);
    if (start_pos == std::string::npos)
        return false;
    str.replace(start_pos, from.length(), to);
    return true;
}

int main()
{
    std::string dir = R"(C:\temp\anim_obj\)";
    std::string meshPath = R"(C:\temp\anim_obj\0001.obj)";
    std::string texturePath = R"(C:\temp\anim_obj\0001.jpg)";
    std::string matPath = R"(C:\temp\anim_obj\0001.obj.mtl)";

    std::vector<char> mesh, texture, mat;
    readAllBytes(meshPath, mesh);
    readAllBytes(texturePath, texture);
    readAllBytes(matPath, mat);

    for (int copy = 2; copy <= 167; ++copy)
    {
        std::stringstream namePref;
        namePref << std::setw(4) << std::setfill('0') << copy;

        auto meshContent = std::string(mesh.begin(), mesh.end());
        replace(meshContent, "0001.obj.mtl", namePref.str() + ".obj.mtl");
        std::ofstream meshCopy{ dir + namePref.str() + ".obj", std::ios::out | std::ios::binary };
        meshCopy.write(meshContent.c_str(), meshContent.length());
        
        std::ofstream texCopy{ dir + namePref.str() + ".jpg", std::ios::out | std::ios::binary };
        texCopy.write(texture.data(), texture.size());

        auto matName = dir + namePref.str() + ".obj.mtl";
        auto matContent = std::string(mat.begin(), mat.end());
        replace(matContent, "0001.jpg", namePref.str() + ".jpg");
        std::ofstream matCopy{ matName, std::ios::out | std::ios::binary };
        matCopy.write(matContent.c_str(), matContent.length());
    }

    return 0;
}
