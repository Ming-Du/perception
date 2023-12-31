# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.21

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
CMAKE_COMMAND = /opt/cmake-3.21.4/bin/cmake

# The command to remove a file.
RM = /opt/cmake-3.21.4/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/xml/git_code/frustum_rcnn/trt_plugin

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/xml/git_code/frustum_rcnn/trt_plugin/build

# Include any dependencies generated for this target.
include CMakeFiles/trt_plugin.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/trt_plugin.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/trt_plugin.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/trt_plugin.dir/flags.make

CMakeFiles/trt_plugin.dir/InferPlugin.cpp.o: CMakeFiles/trt_plugin.dir/flags.make
CMakeFiles/trt_plugin.dir/InferPlugin.cpp.o: ../InferPlugin.cpp
CMakeFiles/trt_plugin.dir/InferPlugin.cpp.o: CMakeFiles/trt_plugin.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/xml/git_code/frustum_rcnn/trt_plugin/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/trt_plugin.dir/InferPlugin.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/trt_plugin.dir/InferPlugin.cpp.o -MF CMakeFiles/trt_plugin.dir/InferPlugin.cpp.o.d -o CMakeFiles/trt_plugin.dir/InferPlugin.cpp.o -c /home/xml/git_code/frustum_rcnn/trt_plugin/InferPlugin.cpp

CMakeFiles/trt_plugin.dir/InferPlugin.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/trt_plugin.dir/InferPlugin.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/xml/git_code/frustum_rcnn/trt_plugin/InferPlugin.cpp > CMakeFiles/trt_plugin.dir/InferPlugin.cpp.i

CMakeFiles/trt_plugin.dir/InferPlugin.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/trt_plugin.dir/InferPlugin.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/xml/git_code/frustum_rcnn/trt_plugin/InferPlugin.cpp -o CMakeFiles/trt_plugin.dir/InferPlugin.cpp.s

# Object files for target trt_plugin
trt_plugin_OBJECTS = \
"CMakeFiles/trt_plugin.dir/InferPlugin.cpp.o"

# External object files for target trt_plugin
trt_plugin_EXTERNAL_OBJECTS =

libtrt_plugin.so: CMakeFiles/trt_plugin.dir/InferPlugin.cpp.o
libtrt_plugin.so: CMakeFiles/trt_plugin.dir/build.make
libtrt_plugin.so: /usr/local/cuda/lib64/libcudart_static.a
libtrt_plugin.so: /usr/lib/x86_64-linux-gnu/librt.so
libtrt_plugin.so: /usr/lib/libnvinfer.so
libtrt_plugin.so: /usr/lib/libnvparsers.so
libtrt_plugin.so: /usr/lib/libnvinfer_plugin.so
libtrt_plugin.so: /usr/local/lib/libnvonnxparser.so
libtrt_plugin.so: FrcnnSegProposalPlugin/libFrcnnSegProposalPlugin_static.a
libtrt_plugin.so: CenterShiftCustomPlugin/libCenterShiftCustomPlugin_static.a
libtrt_plugin.so: /usr/local/cuda/lib64/libcudart_static.a
libtrt_plugin.so: /usr/lib/x86_64-linux-gnu/librt.so
libtrt_plugin.so: /usr/lib/libnvinfer.so
libtrt_plugin.so: /usr/lib/libnvparsers.so
libtrt_plugin.so: /usr/lib/libnvinfer_plugin.so
libtrt_plugin.so: /usr/local/lib/libnvonnxparser.so
libtrt_plugin.so: CMakeFiles/trt_plugin.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/xml/git_code/frustum_rcnn/trt_plugin/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX shared library libtrt_plugin.so"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/trt_plugin.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/trt_plugin.dir/build: libtrt_plugin.so
.PHONY : CMakeFiles/trt_plugin.dir/build

CMakeFiles/trt_plugin.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/trt_plugin.dir/cmake_clean.cmake
.PHONY : CMakeFiles/trt_plugin.dir/clean

CMakeFiles/trt_plugin.dir/depend:
	cd /home/xml/git_code/frustum_rcnn/trt_plugin/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/xml/git_code/frustum_rcnn/trt_plugin /home/xml/git_code/frustum_rcnn/trt_plugin /home/xml/git_code/frustum_rcnn/trt_plugin/build /home/xml/git_code/frustum_rcnn/trt_plugin/build /home/xml/git_code/frustum_rcnn/trt_plugin/build/CMakeFiles/trt_plugin.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/trt_plugin.dir/depend

