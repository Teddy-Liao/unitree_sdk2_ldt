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
CMAKE_SOURCE_DIR = /home/parallels/Projects/unitree_sdk2_ldt

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/parallels/Projects/unitree_sdk2_ldt/build

# Include any dependencies generated for this target.
include CMakeFiles/test_publisher.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/test_publisher.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/test_publisher.dir/flags.make

CMakeFiles/test_publisher.dir/example/helloworld/publisher.cpp.o: CMakeFiles/test_publisher.dir/flags.make
CMakeFiles/test_publisher.dir/example/helloworld/publisher.cpp.o: ../example/helloworld/publisher.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/parallels/Projects/unitree_sdk2_ldt/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/test_publisher.dir/example/helloworld/publisher.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/test_publisher.dir/example/helloworld/publisher.cpp.o -c /home/parallels/Projects/unitree_sdk2_ldt/example/helloworld/publisher.cpp

CMakeFiles/test_publisher.dir/example/helloworld/publisher.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/test_publisher.dir/example/helloworld/publisher.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/parallels/Projects/unitree_sdk2_ldt/example/helloworld/publisher.cpp > CMakeFiles/test_publisher.dir/example/helloworld/publisher.cpp.i

CMakeFiles/test_publisher.dir/example/helloworld/publisher.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/test_publisher.dir/example/helloworld/publisher.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/parallels/Projects/unitree_sdk2_ldt/example/helloworld/publisher.cpp -o CMakeFiles/test_publisher.dir/example/helloworld/publisher.cpp.s

CMakeFiles/test_publisher.dir/example/helloworld/HelloWorldData.cpp.o: CMakeFiles/test_publisher.dir/flags.make
CMakeFiles/test_publisher.dir/example/helloworld/HelloWorldData.cpp.o: ../example/helloworld/HelloWorldData.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/parallels/Projects/unitree_sdk2_ldt/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/test_publisher.dir/example/helloworld/HelloWorldData.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/test_publisher.dir/example/helloworld/HelloWorldData.cpp.o -c /home/parallels/Projects/unitree_sdk2_ldt/example/helloworld/HelloWorldData.cpp

CMakeFiles/test_publisher.dir/example/helloworld/HelloWorldData.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/test_publisher.dir/example/helloworld/HelloWorldData.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/parallels/Projects/unitree_sdk2_ldt/example/helloworld/HelloWorldData.cpp > CMakeFiles/test_publisher.dir/example/helloworld/HelloWorldData.cpp.i

CMakeFiles/test_publisher.dir/example/helloworld/HelloWorldData.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/test_publisher.dir/example/helloworld/HelloWorldData.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/parallels/Projects/unitree_sdk2_ldt/example/helloworld/HelloWorldData.cpp -o CMakeFiles/test_publisher.dir/example/helloworld/HelloWorldData.cpp.s

# Object files for target test_publisher
test_publisher_OBJECTS = \
"CMakeFiles/test_publisher.dir/example/helloworld/publisher.cpp.o" \
"CMakeFiles/test_publisher.dir/example/helloworld/HelloWorldData.cpp.o"

# External object files for target test_publisher
test_publisher_EXTERNAL_OBJECTS =

test_publisher: CMakeFiles/test_publisher.dir/example/helloworld/publisher.cpp.o
test_publisher: CMakeFiles/test_publisher.dir/example/helloworld/HelloWorldData.cpp.o
test_publisher: CMakeFiles/test_publisher.dir/build.make
test_publisher: CMakeFiles/test_publisher.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/parallels/Projects/unitree_sdk2_ldt/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX executable test_publisher"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/test_publisher.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/test_publisher.dir/build: test_publisher

.PHONY : CMakeFiles/test_publisher.dir/build

CMakeFiles/test_publisher.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/test_publisher.dir/cmake_clean.cmake
.PHONY : CMakeFiles/test_publisher.dir/clean

CMakeFiles/test_publisher.dir/depend:
	cd /home/parallels/Projects/unitree_sdk2_ldt/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/parallels/Projects/unitree_sdk2_ldt /home/parallels/Projects/unitree_sdk2_ldt /home/parallels/Projects/unitree_sdk2_ldt/build /home/parallels/Projects/unitree_sdk2_ldt/build /home/parallels/Projects/unitree_sdk2_ldt/build/CMakeFiles/test_publisher.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/test_publisher.dir/depend

