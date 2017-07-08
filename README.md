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

Add `RSSDK_DIR` part only if you need RealSense grabbing support.

Theoretically the code should also build for Linux and Mac (maybe with a couple of adjustment here and there). Just replace paths to libraries with the appropriate ones for your system.

If you happen to use my code in any of your projects, I would really appreciate if you write me a letter.
If you have any questions or problems with the code please also feel free to reach me: apetrenko1991@gmail.com

