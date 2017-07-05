# 4dvideo

Capturing volumetric videos with Google Tango, RealSense R200 and fast Delaunay triangulation.

To download 3rd party submodules:

```
git submodule update --init --recursive
```

CMake command line for Visual Studio 2015:

```cmake
cmake -G"Visual Studio 14 2015 Win64" -DOpenCV_DIR=C:/all/projects/libs/opencv-3.2.0/build/install -DGLFW_ROOT_DIR=C:/all/projects/libs/glfw-3.2.1.bin.WIN64 -DGLEW_LIBRARY=C:/all/projects/libs/glew-2.0.0/lib/Release/x64/glew32.lib -DGLEW_INCLUDE_DIR=C:/all/projects/libs/glew-2.0.0/include -DRSSDK_DIR="C:/Program Files (x86)/Intel/RSSDK" ../..
```