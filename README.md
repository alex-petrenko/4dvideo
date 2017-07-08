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

Project structure:
* `src/libs`: reusable library modules
  * `lib4d`: library module with utilities for 4d content processing, including dataset reading and writing, rendering, meshing and filtering
  * `librealsense`: Intel RealSense functionality, including the grabbing code
  * `libtri`: Delaunay triangulation implementation
  * `libutil`: various utilities, including logging, profiling, geometric primitives, .ply IO etc.
* `src/apps`: all the apps
  * `4d_player_app <4dv-dataset-path>` Playbacks 4D "movies" in .4dv binary format
  * `animation_writer_app <4dv-dataset-path> <timeframe-anim-directory>` Converts binary .4dv movie into a series of .ply meshes for every frame. Once zipped this can be uploaded to Sketchfab (this format is called "timeframe animation"). The directory must exist beforehand.
  * `realsense_grabber_app <output-4dv-file>` Captures 4D movie from Intel RealSense in .4dv format.
  * `triangulation_visualizer_app`: the app I used to generate GIF visualizations of Delaunay triangulation algorithm. Must enable `WITH_VIS` preprocessor variable for it to work.
* `src/test`: some unit tests created with awesome GTest library.
* `android_src`: all the Java code for Google Tango, mostly an unfinished binary grabber for Tango

If you happen to use my code in any of your projects, I would really appreciate if you write me a letter.
If you have any questions or problems with the code please also feel free to reach me: apetrenko1991@gmail.com

