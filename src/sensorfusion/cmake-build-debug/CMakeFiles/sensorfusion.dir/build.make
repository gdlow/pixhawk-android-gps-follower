# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.14

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
CMAKE_COMMAND = /snap/clion/73/bin/cmake/linux/bin/cmake

# The command to remove a file.
RM = /snap/clion/73/bin/cmake/linux/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/geraldlow/Misc/gdp/sensorfusion

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/geraldlow/Misc/gdp/sensorfusion/cmake-build-debug

# Include any dependencies generated for this target.
include CMakeFiles/sensorfusion.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/sensorfusion.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/sensorfusion.dir/flags.make

CMakeFiles/sensorfusion.dir/GPSAccKalman.cpp.o: CMakeFiles/sensorfusion.dir/flags.make
CMakeFiles/sensorfusion.dir/GPSAccKalman.cpp.o: ../GPSAccKalman.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/geraldlow/Misc/gdp/sensorfusion/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/sensorfusion.dir/GPSAccKalman.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/sensorfusion.dir/GPSAccKalman.cpp.o -c /home/geraldlow/Misc/gdp/sensorfusion/GPSAccKalman.cpp

CMakeFiles/sensorfusion.dir/GPSAccKalman.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/sensorfusion.dir/GPSAccKalman.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/geraldlow/Misc/gdp/sensorfusion/GPSAccKalman.cpp > CMakeFiles/sensorfusion.dir/GPSAccKalman.cpp.i

CMakeFiles/sensorfusion.dir/GPSAccKalman.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/sensorfusion.dir/GPSAccKalman.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/geraldlow/Misc/gdp/sensorfusion/GPSAccKalman.cpp -o CMakeFiles/sensorfusion.dir/GPSAccKalman.cpp.s

CMakeFiles/sensorfusion.dir/Kalman.cpp.o: CMakeFiles/sensorfusion.dir/flags.make
CMakeFiles/sensorfusion.dir/Kalman.cpp.o: ../Kalman.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/geraldlow/Misc/gdp/sensorfusion/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/sensorfusion.dir/Kalman.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/sensorfusion.dir/Kalman.cpp.o -c /home/geraldlow/Misc/gdp/sensorfusion/Kalman.cpp

CMakeFiles/sensorfusion.dir/Kalman.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/sensorfusion.dir/Kalman.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/geraldlow/Misc/gdp/sensorfusion/Kalman.cpp > CMakeFiles/sensorfusion.dir/Kalman.cpp.i

CMakeFiles/sensorfusion.dir/Kalman.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/sensorfusion.dir/Kalman.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/geraldlow/Misc/gdp/sensorfusion/Kalman.cpp -o CMakeFiles/sensorfusion.dir/Kalman.cpp.s

CMakeFiles/sensorfusion.dir/Matrix.cpp.o: CMakeFiles/sensorfusion.dir/flags.make
CMakeFiles/sensorfusion.dir/Matrix.cpp.o: ../Matrix.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/geraldlow/Misc/gdp/sensorfusion/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/sensorfusion.dir/Matrix.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/sensorfusion.dir/Matrix.cpp.o -c /home/geraldlow/Misc/gdp/sensorfusion/Matrix.cpp

CMakeFiles/sensorfusion.dir/Matrix.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/sensorfusion.dir/Matrix.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/geraldlow/Misc/gdp/sensorfusion/Matrix.cpp > CMakeFiles/sensorfusion.dir/Matrix.cpp.i

CMakeFiles/sensorfusion.dir/Matrix.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/sensorfusion.dir/Matrix.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/geraldlow/Misc/gdp/sensorfusion/Matrix.cpp -o CMakeFiles/sensorfusion.dir/Matrix.cpp.s

# Object files for target sensorfusion
sensorfusion_OBJECTS = \
"CMakeFiles/sensorfusion.dir/GPSAccKalman.cpp.o" \
"CMakeFiles/sensorfusion.dir/Kalman.cpp.o" \
"CMakeFiles/sensorfusion.dir/Matrix.cpp.o"

# External object files for target sensorfusion
sensorfusion_EXTERNAL_OBJECTS =

libsensorfusion.a: CMakeFiles/sensorfusion.dir/GPSAccKalman.cpp.o
libsensorfusion.a: CMakeFiles/sensorfusion.dir/Kalman.cpp.o
libsensorfusion.a: CMakeFiles/sensorfusion.dir/Matrix.cpp.o
libsensorfusion.a: CMakeFiles/sensorfusion.dir/build.make
libsensorfusion.a: CMakeFiles/sensorfusion.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/geraldlow/Misc/gdp/sensorfusion/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Linking CXX static library libsensorfusion.a"
	$(CMAKE_COMMAND) -P CMakeFiles/sensorfusion.dir/cmake_clean_target.cmake
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/sensorfusion.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/sensorfusion.dir/build: libsensorfusion.a

.PHONY : CMakeFiles/sensorfusion.dir/build

CMakeFiles/sensorfusion.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/sensorfusion.dir/cmake_clean.cmake
.PHONY : CMakeFiles/sensorfusion.dir/clean

CMakeFiles/sensorfusion.dir/depend:
	cd /home/geraldlow/Misc/gdp/sensorfusion/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/geraldlow/Misc/gdp/sensorfusion /home/geraldlow/Misc/gdp/sensorfusion /home/geraldlow/Misc/gdp/sensorfusion/cmake-build-debug /home/geraldlow/Misc/gdp/sensorfusion/cmake-build-debug /home/geraldlow/Misc/gdp/sensorfusion/cmake-build-debug/CMakeFiles/sensorfusion.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/sensorfusion.dir/depend

