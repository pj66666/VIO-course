# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.16

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list


# Suppress display of executed commands.
$(VERBOSE).SILENT:


# A target that is always out of date.
cmake_force:

.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/pj/pj/vio_with_only_eigen/imu_allan/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/pj/pj/vio_with_only_eigen/imu_allan/build

# Include any dependencies generated for this target.
include code_utils/CMakeFiles/pnp.dir/depend.make

# Include the progress variables for this target.
include code_utils/CMakeFiles/pnp.dir/progress.make

# Include the compile flags for this target's objects.
include code_utils/CMakeFiles/pnp.dir/flags.make

code_utils/CMakeFiles/pnp.dir/src/cv_utils/dlt/dlt.cpp.o: code_utils/CMakeFiles/pnp.dir/flags.make
code_utils/CMakeFiles/pnp.dir/src/cv_utils/dlt/dlt.cpp.o: /home/pj/pj/vio_with_only_eigen/imu_allan/src/code_utils/src/cv_utils/dlt/dlt.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pj/pj/vio_with_only_eigen/imu_allan/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object code_utils/CMakeFiles/pnp.dir/src/cv_utils/dlt/dlt.cpp.o"
	cd /home/pj/pj/vio_with_only_eigen/imu_allan/build/code_utils && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/pnp.dir/src/cv_utils/dlt/dlt.cpp.o -c /home/pj/pj/vio_with_only_eigen/imu_allan/src/code_utils/src/cv_utils/dlt/dlt.cpp

code_utils/CMakeFiles/pnp.dir/src/cv_utils/dlt/dlt.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/pnp.dir/src/cv_utils/dlt/dlt.cpp.i"
	cd /home/pj/pj/vio_with_only_eigen/imu_allan/build/code_utils && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pj/pj/vio_with_only_eigen/imu_allan/src/code_utils/src/cv_utils/dlt/dlt.cpp > CMakeFiles/pnp.dir/src/cv_utils/dlt/dlt.cpp.i

code_utils/CMakeFiles/pnp.dir/src/cv_utils/dlt/dlt.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/pnp.dir/src/cv_utils/dlt/dlt.cpp.s"
	cd /home/pj/pj/vio_with_only_eigen/imu_allan/build/code_utils && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pj/pj/vio_with_only_eigen/imu_allan/src/code_utils/src/cv_utils/dlt/dlt.cpp -o CMakeFiles/pnp.dir/src/cv_utils/dlt/dlt.cpp.s

code_utils/CMakeFiles/pnp.dir/src/cv_utils/pnp/pnp.cpp.o: code_utils/CMakeFiles/pnp.dir/flags.make
code_utils/CMakeFiles/pnp.dir/src/cv_utils/pnp/pnp.cpp.o: /home/pj/pj/vio_with_only_eigen/imu_allan/src/code_utils/src/cv_utils/pnp/pnp.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pj/pj/vio_with_only_eigen/imu_allan/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object code_utils/CMakeFiles/pnp.dir/src/cv_utils/pnp/pnp.cpp.o"
	cd /home/pj/pj/vio_with_only_eigen/imu_allan/build/code_utils && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/pnp.dir/src/cv_utils/pnp/pnp.cpp.o -c /home/pj/pj/vio_with_only_eigen/imu_allan/src/code_utils/src/cv_utils/pnp/pnp.cpp

code_utils/CMakeFiles/pnp.dir/src/cv_utils/pnp/pnp.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/pnp.dir/src/cv_utils/pnp/pnp.cpp.i"
	cd /home/pj/pj/vio_with_only_eigen/imu_allan/build/code_utils && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pj/pj/vio_with_only_eigen/imu_allan/src/code_utils/src/cv_utils/pnp/pnp.cpp > CMakeFiles/pnp.dir/src/cv_utils/pnp/pnp.cpp.i

code_utils/CMakeFiles/pnp.dir/src/cv_utils/pnp/pnp.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/pnp.dir/src/cv_utils/pnp/pnp.cpp.s"
	cd /home/pj/pj/vio_with_only_eigen/imu_allan/build/code_utils && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pj/pj/vio_with_only_eigen/imu_allan/src/code_utils/src/cv_utils/pnp/pnp.cpp -o CMakeFiles/pnp.dir/src/cv_utils/pnp/pnp.cpp.s

code_utils/CMakeFiles/pnp.dir/src/cv_utils/pnp/linearpnp.cpp.o: code_utils/CMakeFiles/pnp.dir/flags.make
code_utils/CMakeFiles/pnp.dir/src/cv_utils/pnp/linearpnp.cpp.o: /home/pj/pj/vio_with_only_eigen/imu_allan/src/code_utils/src/cv_utils/pnp/linearpnp.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pj/pj/vio_with_only_eigen/imu_allan/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object code_utils/CMakeFiles/pnp.dir/src/cv_utils/pnp/linearpnp.cpp.o"
	cd /home/pj/pj/vio_with_only_eigen/imu_allan/build/code_utils && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/pnp.dir/src/cv_utils/pnp/linearpnp.cpp.o -c /home/pj/pj/vio_with_only_eigen/imu_allan/src/code_utils/src/cv_utils/pnp/linearpnp.cpp

code_utils/CMakeFiles/pnp.dir/src/cv_utils/pnp/linearpnp.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/pnp.dir/src/cv_utils/pnp/linearpnp.cpp.i"
	cd /home/pj/pj/vio_with_only_eigen/imu_allan/build/code_utils && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pj/pj/vio_with_only_eigen/imu_allan/src/code_utils/src/cv_utils/pnp/linearpnp.cpp > CMakeFiles/pnp.dir/src/cv_utils/pnp/linearpnp.cpp.i

code_utils/CMakeFiles/pnp.dir/src/cv_utils/pnp/linearpnp.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/pnp.dir/src/cv_utils/pnp/linearpnp.cpp.s"
	cd /home/pj/pj/vio_with_only_eigen/imu_allan/build/code_utils && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pj/pj/vio_with_only_eigen/imu_allan/src/code_utils/src/cv_utils/pnp/linearpnp.cpp -o CMakeFiles/pnp.dir/src/cv_utils/pnp/linearpnp.cpp.s

code_utils/CMakeFiles/pnp.dir/src/cv_utils/pnp/nonlinearpnp.cpp.o: code_utils/CMakeFiles/pnp.dir/flags.make
code_utils/CMakeFiles/pnp.dir/src/cv_utils/pnp/nonlinearpnp.cpp.o: /home/pj/pj/vio_with_only_eigen/imu_allan/src/code_utils/src/cv_utils/pnp/nonlinearpnp.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pj/pj/vio_with_only_eigen/imu_allan/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object code_utils/CMakeFiles/pnp.dir/src/cv_utils/pnp/nonlinearpnp.cpp.o"
	cd /home/pj/pj/vio_with_only_eigen/imu_allan/build/code_utils && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/pnp.dir/src/cv_utils/pnp/nonlinearpnp.cpp.o -c /home/pj/pj/vio_with_only_eigen/imu_allan/src/code_utils/src/cv_utils/pnp/nonlinearpnp.cpp

code_utils/CMakeFiles/pnp.dir/src/cv_utils/pnp/nonlinearpnp.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/pnp.dir/src/cv_utils/pnp/nonlinearpnp.cpp.i"
	cd /home/pj/pj/vio_with_only_eigen/imu_allan/build/code_utils && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pj/pj/vio_with_only_eigen/imu_allan/src/code_utils/src/cv_utils/pnp/nonlinearpnp.cpp > CMakeFiles/pnp.dir/src/cv_utils/pnp/nonlinearpnp.cpp.i

code_utils/CMakeFiles/pnp.dir/src/cv_utils/pnp/nonlinearpnp.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/pnp.dir/src/cv_utils/pnp/nonlinearpnp.cpp.s"
	cd /home/pj/pj/vio_with_only_eigen/imu_allan/build/code_utils && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pj/pj/vio_with_only_eigen/imu_allan/src/code_utils/src/cv_utils/pnp/nonlinearpnp.cpp -o CMakeFiles/pnp.dir/src/cv_utils/pnp/nonlinearpnp.cpp.s

# Object files for target pnp
pnp_OBJECTS = \
"CMakeFiles/pnp.dir/src/cv_utils/dlt/dlt.cpp.o" \
"CMakeFiles/pnp.dir/src/cv_utils/pnp/pnp.cpp.o" \
"CMakeFiles/pnp.dir/src/cv_utils/pnp/linearpnp.cpp.o" \
"CMakeFiles/pnp.dir/src/cv_utils/pnp/nonlinearpnp.cpp.o"

# External object files for target pnp
pnp_EXTERNAL_OBJECTS =

/home/pj/pj/vio_with_only_eigen/imu_allan/devel/lib/libpnp.so: code_utils/CMakeFiles/pnp.dir/src/cv_utils/dlt/dlt.cpp.o
/home/pj/pj/vio_with_only_eigen/imu_allan/devel/lib/libpnp.so: code_utils/CMakeFiles/pnp.dir/src/cv_utils/pnp/pnp.cpp.o
/home/pj/pj/vio_with_only_eigen/imu_allan/devel/lib/libpnp.so: code_utils/CMakeFiles/pnp.dir/src/cv_utils/pnp/linearpnp.cpp.o
/home/pj/pj/vio_with_only_eigen/imu_allan/devel/lib/libpnp.so: code_utils/CMakeFiles/pnp.dir/src/cv_utils/pnp/nonlinearpnp.cpp.o
/home/pj/pj/vio_with_only_eigen/imu_allan/devel/lib/libpnp.so: code_utils/CMakeFiles/pnp.dir/build.make
/home/pj/pj/vio_with_only_eigen/imu_allan/devel/lib/libpnp.so: /usr/local/lib/libopencv_dnn.so.3.4.16
/home/pj/pj/vio_with_only_eigen/imu_allan/devel/lib/libpnp.so: /usr/local/lib/libopencv_highgui.so.3.4.16
/home/pj/pj/vio_with_only_eigen/imu_allan/devel/lib/libpnp.so: /usr/local/lib/libopencv_ml.so.3.4.16
/home/pj/pj/vio_with_only_eigen/imu_allan/devel/lib/libpnp.so: /usr/local/lib/libopencv_objdetect.so.3.4.16
/home/pj/pj/vio_with_only_eigen/imu_allan/devel/lib/libpnp.so: /usr/local/lib/libopencv_shape.so.3.4.16
/home/pj/pj/vio_with_only_eigen/imu_allan/devel/lib/libpnp.so: /usr/local/lib/libopencv_stitching.so.3.4.16
/home/pj/pj/vio_with_only_eigen/imu_allan/devel/lib/libpnp.so: /usr/local/lib/libopencv_superres.so.3.4.16
/home/pj/pj/vio_with_only_eigen/imu_allan/devel/lib/libpnp.so: /usr/local/lib/libopencv_videostab.so.3.4.16
/home/pj/pj/vio_with_only_eigen/imu_allan/devel/lib/libpnp.so: /usr/local/lib/libopencv_viz.so.3.4.16
/home/pj/pj/vio_with_only_eigen/imu_allan/devel/lib/libpnp.so: /usr/local/lib/libceres.a
/home/pj/pj/vio_with_only_eigen/imu_allan/devel/lib/libpnp.so: /usr/local/lib/libopencv_calib3d.so.3.4.16
/home/pj/pj/vio_with_only_eigen/imu_allan/devel/lib/libpnp.so: /usr/local/lib/libopencv_features2d.so.3.4.16
/home/pj/pj/vio_with_only_eigen/imu_allan/devel/lib/libpnp.so: /usr/local/lib/libopencv_flann.so.3.4.16
/home/pj/pj/vio_with_only_eigen/imu_allan/devel/lib/libpnp.so: /usr/local/lib/libopencv_photo.so.3.4.16
/home/pj/pj/vio_with_only_eigen/imu_allan/devel/lib/libpnp.so: /usr/local/lib/libopencv_video.so.3.4.16
/home/pj/pj/vio_with_only_eigen/imu_allan/devel/lib/libpnp.so: /usr/local/lib/libopencv_videoio.so.3.4.16
/home/pj/pj/vio_with_only_eigen/imu_allan/devel/lib/libpnp.so: /usr/local/lib/libopencv_imgcodecs.so.3.4.16
/home/pj/pj/vio_with_only_eigen/imu_allan/devel/lib/libpnp.so: /usr/local/lib/libopencv_imgproc.so.3.4.16
/home/pj/pj/vio_with_only_eigen/imu_allan/devel/lib/libpnp.so: /usr/local/lib/libopencv_core.so.3.4.16
/home/pj/pj/vio_with_only_eigen/imu_allan/devel/lib/libpnp.so: /usr/lib/x86_64-linux-gnu/libglog.so
/home/pj/pj/vio_with_only_eigen/imu_allan/devel/lib/libpnp.so: /usr/lib/x86_64-linux-gnu/libgflags.so.2.2.2
/home/pj/pj/vio_with_only_eigen/imu_allan/devel/lib/libpnp.so: /usr/lib/x86_64-linux-gnu/libspqr.so
/home/pj/pj/vio_with_only_eigen/imu_allan/devel/lib/libpnp.so: /usr/lib/x86_64-linux-gnu/libtbb.so
/home/pj/pj/vio_with_only_eigen/imu_allan/devel/lib/libpnp.so: /usr/lib/x86_64-linux-gnu/libtbbmalloc.so
/home/pj/pj/vio_with_only_eigen/imu_allan/devel/lib/libpnp.so: /usr/lib/x86_64-linux-gnu/libcholmod.so
/home/pj/pj/vio_with_only_eigen/imu_allan/devel/lib/libpnp.so: /usr/lib/x86_64-linux-gnu/libccolamd.so
/home/pj/pj/vio_with_only_eigen/imu_allan/devel/lib/libpnp.so: /usr/lib/x86_64-linux-gnu/libcamd.so
/home/pj/pj/vio_with_only_eigen/imu_allan/devel/lib/libpnp.so: /usr/lib/x86_64-linux-gnu/libcolamd.so
/home/pj/pj/vio_with_only_eigen/imu_allan/devel/lib/libpnp.so: /usr/lib/x86_64-linux-gnu/libamd.so
/home/pj/pj/vio_with_only_eigen/imu_allan/devel/lib/libpnp.so: /usr/lib/x86_64-linux-gnu/liblapack.so
/home/pj/pj/vio_with_only_eigen/imu_allan/devel/lib/libpnp.so: /usr/lib/x86_64-linux-gnu/libf77blas.so
/home/pj/pj/vio_with_only_eigen/imu_allan/devel/lib/libpnp.so: /usr/lib/x86_64-linux-gnu/libatlas.so
/home/pj/pj/vio_with_only_eigen/imu_allan/devel/lib/libpnp.so: /usr/lib/x86_64-linux-gnu/libsuitesparseconfig.so
/home/pj/pj/vio_with_only_eigen/imu_allan/devel/lib/libpnp.so: /usr/lib/x86_64-linux-gnu/librt.so
/home/pj/pj/vio_with_only_eigen/imu_allan/devel/lib/libpnp.so: /usr/lib/x86_64-linux-gnu/liblapack.so
/home/pj/pj/vio_with_only_eigen/imu_allan/devel/lib/libpnp.so: /usr/lib/x86_64-linux-gnu/libf77blas.so
/home/pj/pj/vio_with_only_eigen/imu_allan/devel/lib/libpnp.so: /usr/lib/x86_64-linux-gnu/libatlas.so
/home/pj/pj/vio_with_only_eigen/imu_allan/devel/lib/libpnp.so: /usr/lib/x86_64-linux-gnu/libsuitesparseconfig.so
/home/pj/pj/vio_with_only_eigen/imu_allan/devel/lib/libpnp.so: /usr/lib/x86_64-linux-gnu/librt.so
/home/pj/pj/vio_with_only_eigen/imu_allan/devel/lib/libpnp.so: code_utils/CMakeFiles/pnp.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/pj/pj/vio_with_only_eigen/imu_allan/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Linking CXX shared library /home/pj/pj/vio_with_only_eigen/imu_allan/devel/lib/libpnp.so"
	cd /home/pj/pj/vio_with_only_eigen/imu_allan/build/code_utils && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/pnp.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
code_utils/CMakeFiles/pnp.dir/build: /home/pj/pj/vio_with_only_eigen/imu_allan/devel/lib/libpnp.so

.PHONY : code_utils/CMakeFiles/pnp.dir/build

code_utils/CMakeFiles/pnp.dir/clean:
	cd /home/pj/pj/vio_with_only_eigen/imu_allan/build/code_utils && $(CMAKE_COMMAND) -P CMakeFiles/pnp.dir/cmake_clean.cmake
.PHONY : code_utils/CMakeFiles/pnp.dir/clean

code_utils/CMakeFiles/pnp.dir/depend:
	cd /home/pj/pj/vio_with_only_eigen/imu_allan/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/pj/pj/vio_with_only_eigen/imu_allan/src /home/pj/pj/vio_with_only_eigen/imu_allan/src/code_utils /home/pj/pj/vio_with_only_eigen/imu_allan/build /home/pj/pj/vio_with_only_eigen/imu_allan/build/code_utils /home/pj/pj/vio_with_only_eigen/imu_allan/build/code_utils/CMakeFiles/pnp.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : code_utils/CMakeFiles/pnp.dir/depend

