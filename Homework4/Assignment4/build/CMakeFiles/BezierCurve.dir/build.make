# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.24

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Disable VCS-based implicit rules.
% : %,v

# Disable VCS-based implicit rules.
% : RCS/%

# Disable VCS-based implicit rules.
% : RCS/%,v

# Disable VCS-based implicit rules.
% : SCCS/s.%

# Disable VCS-based implicit rules.
% : s.%

.SUFFIXES: .hpux_make_needs_suffix_list

# Command-line flag to silence nested $(MAKE).
$(VERBOSE)MAKESILENT = -s

#Suppress display of executed commands.
$(VERBOSE).SILENT:

# A target that is always out of date.
cmake_force:
.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /opt/homebrew/Cellar/cmake/3.24.0/bin/cmake

# The command to remove a file.
RM = /opt/homebrew/Cellar/cmake/3.24.0/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /Users/xiaohuo/Documents/GAMES101/Homework4/Assignment4

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /Users/xiaohuo/Documents/GAMES101/Homework4/Assignment4/build

# Include any dependencies generated for this target.
include CMakeFiles/BezierCurve.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/BezierCurve.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/BezierCurve.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/BezierCurve.dir/flags.make

CMakeFiles/BezierCurve.dir/main.cpp.o: CMakeFiles/BezierCurve.dir/flags.make
CMakeFiles/BezierCurve.dir/main.cpp.o: /Users/xiaohuo/Documents/GAMES101/Homework4/Assignment4/main.cpp
CMakeFiles/BezierCurve.dir/main.cpp.o: CMakeFiles/BezierCurve.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/xiaohuo/Documents/GAMES101/Homework4/Assignment4/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/BezierCurve.dir/main.cpp.o"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/BezierCurve.dir/main.cpp.o -MF CMakeFiles/BezierCurve.dir/main.cpp.o.d -o CMakeFiles/BezierCurve.dir/main.cpp.o -c /Users/xiaohuo/Documents/GAMES101/Homework4/Assignment4/main.cpp

CMakeFiles/BezierCurve.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/BezierCurve.dir/main.cpp.i"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/xiaohuo/Documents/GAMES101/Homework4/Assignment4/main.cpp > CMakeFiles/BezierCurve.dir/main.cpp.i

CMakeFiles/BezierCurve.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/BezierCurve.dir/main.cpp.s"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/xiaohuo/Documents/GAMES101/Homework4/Assignment4/main.cpp -o CMakeFiles/BezierCurve.dir/main.cpp.s

# Object files for target BezierCurve
BezierCurve_OBJECTS = \
"CMakeFiles/BezierCurve.dir/main.cpp.o"

# External object files for target BezierCurve
BezierCurve_EXTERNAL_OBJECTS =

BezierCurve: CMakeFiles/BezierCurve.dir/main.cpp.o
BezierCurve: CMakeFiles/BezierCurve.dir/build.make
BezierCurve: /opt/homebrew/lib/libopencv_gapi.4.5.5.dylib
BezierCurve: /opt/homebrew/lib/libopencv_stitching.4.5.5.dylib
BezierCurve: /opt/homebrew/lib/libopencv_alphamat.4.5.5.dylib
BezierCurve: /opt/homebrew/lib/libopencv_aruco.4.5.5.dylib
BezierCurve: /opt/homebrew/lib/libopencv_barcode.4.5.5.dylib
BezierCurve: /opt/homebrew/lib/libopencv_bgsegm.4.5.5.dylib
BezierCurve: /opt/homebrew/lib/libopencv_bioinspired.4.5.5.dylib
BezierCurve: /opt/homebrew/lib/libopencv_ccalib.4.5.5.dylib
BezierCurve: /opt/homebrew/lib/libopencv_dnn_objdetect.4.5.5.dylib
BezierCurve: /opt/homebrew/lib/libopencv_dnn_superres.4.5.5.dylib
BezierCurve: /opt/homebrew/lib/libopencv_dpm.4.5.5.dylib
BezierCurve: /opt/homebrew/lib/libopencv_face.4.5.5.dylib
BezierCurve: /opt/homebrew/lib/libopencv_freetype.4.5.5.dylib
BezierCurve: /opt/homebrew/lib/libopencv_fuzzy.4.5.5.dylib
BezierCurve: /opt/homebrew/lib/libopencv_hfs.4.5.5.dylib
BezierCurve: /opt/homebrew/lib/libopencv_img_hash.4.5.5.dylib
BezierCurve: /opt/homebrew/lib/libopencv_intensity_transform.4.5.5.dylib
BezierCurve: /opt/homebrew/lib/libopencv_line_descriptor.4.5.5.dylib
BezierCurve: /opt/homebrew/lib/libopencv_mcc.4.5.5.dylib
BezierCurve: /opt/homebrew/lib/libopencv_quality.4.5.5.dylib
BezierCurve: /opt/homebrew/lib/libopencv_rapid.4.5.5.dylib
BezierCurve: /opt/homebrew/lib/libopencv_reg.4.5.5.dylib
BezierCurve: /opt/homebrew/lib/libopencv_rgbd.4.5.5.dylib
BezierCurve: /opt/homebrew/lib/libopencv_saliency.4.5.5.dylib
BezierCurve: /opt/homebrew/lib/libopencv_sfm.4.5.5.dylib
BezierCurve: /opt/homebrew/lib/libopencv_stereo.4.5.5.dylib
BezierCurve: /opt/homebrew/lib/libopencv_structured_light.4.5.5.dylib
BezierCurve: /opt/homebrew/lib/libopencv_superres.4.5.5.dylib
BezierCurve: /opt/homebrew/lib/libopencv_surface_matching.4.5.5.dylib
BezierCurve: /opt/homebrew/lib/libopencv_tracking.4.5.5.dylib
BezierCurve: /opt/homebrew/lib/libopencv_videostab.4.5.5.dylib
BezierCurve: /opt/homebrew/lib/libopencv_viz.4.5.5.dylib
BezierCurve: /opt/homebrew/lib/libopencv_wechat_qrcode.4.5.5.dylib
BezierCurve: /opt/homebrew/lib/libopencv_xfeatures2d.4.5.5.dylib
BezierCurve: /opt/homebrew/lib/libopencv_xobjdetect.4.5.5.dylib
BezierCurve: /opt/homebrew/lib/libopencv_xphoto.4.5.5.dylib
BezierCurve: /opt/homebrew/lib/libopencv_shape.4.5.5.dylib
BezierCurve: /opt/homebrew/lib/libopencv_highgui.4.5.5.dylib
BezierCurve: /opt/homebrew/lib/libopencv_datasets.4.5.5.dylib
BezierCurve: /opt/homebrew/lib/libopencv_plot.4.5.5.dylib
BezierCurve: /opt/homebrew/lib/libopencv_text.4.5.5.dylib
BezierCurve: /opt/homebrew/lib/libopencv_ml.4.5.5.dylib
BezierCurve: /opt/homebrew/lib/libopencv_phase_unwrapping.4.5.5.dylib
BezierCurve: /opt/homebrew/lib/libopencv_optflow.4.5.5.dylib
BezierCurve: /opt/homebrew/lib/libopencv_ximgproc.4.5.5.dylib
BezierCurve: /opt/homebrew/lib/libopencv_video.4.5.5.dylib
BezierCurve: /opt/homebrew/lib/libopencv_videoio.4.5.5.dylib
BezierCurve: /opt/homebrew/lib/libopencv_imgcodecs.4.5.5.dylib
BezierCurve: /opt/homebrew/lib/libopencv_objdetect.4.5.5.dylib
BezierCurve: /opt/homebrew/lib/libopencv_calib3d.4.5.5.dylib
BezierCurve: /opt/homebrew/lib/libopencv_dnn.4.5.5.dylib
BezierCurve: /opt/homebrew/lib/libopencv_features2d.4.5.5.dylib
BezierCurve: /opt/homebrew/lib/libopencv_flann.4.5.5.dylib
BezierCurve: /opt/homebrew/lib/libopencv_photo.4.5.5.dylib
BezierCurve: /opt/homebrew/lib/libopencv_imgproc.4.5.5.dylib
BezierCurve: /opt/homebrew/lib/libopencv_core.4.5.5.dylib
BezierCurve: CMakeFiles/BezierCurve.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/Users/xiaohuo/Documents/GAMES101/Homework4/Assignment4/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable BezierCurve"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/BezierCurve.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/BezierCurve.dir/build: BezierCurve
.PHONY : CMakeFiles/BezierCurve.dir/build

CMakeFiles/BezierCurve.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/BezierCurve.dir/cmake_clean.cmake
.PHONY : CMakeFiles/BezierCurve.dir/clean

CMakeFiles/BezierCurve.dir/depend:
	cd /Users/xiaohuo/Documents/GAMES101/Homework4/Assignment4/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /Users/xiaohuo/Documents/GAMES101/Homework4/Assignment4 /Users/xiaohuo/Documents/GAMES101/Homework4/Assignment4 /Users/xiaohuo/Documents/GAMES101/Homework4/Assignment4/build /Users/xiaohuo/Documents/GAMES101/Homework4/Assignment4/build /Users/xiaohuo/Documents/GAMES101/Homework4/Assignment4/build/CMakeFiles/BezierCurve.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/BezierCurve.dir/depend

