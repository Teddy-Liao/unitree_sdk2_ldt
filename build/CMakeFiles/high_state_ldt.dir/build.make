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
include CMakeFiles/high_state_ldt.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/high_state_ldt.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/high_state_ldt.dir/flags.make

CMakeFiles/high_state_ldt.dir/example/user/high_state_ldt.cpp.o: CMakeFiles/high_state_ldt.dir/flags.make
CMakeFiles/high_state_ldt.dir/example/user/high_state_ldt.cpp.o: ../example/user/high_state_ldt.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/parallels/Projects/unitree_sdk2_ldt/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/high_state_ldt.dir/example/user/high_state_ldt.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/high_state_ldt.dir/example/user/high_state_ldt.cpp.o -c /home/parallels/Projects/unitree_sdk2_ldt/example/user/high_state_ldt.cpp

CMakeFiles/high_state_ldt.dir/example/user/high_state_ldt.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/high_state_ldt.dir/example/user/high_state_ldt.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/parallels/Projects/unitree_sdk2_ldt/example/user/high_state_ldt.cpp > CMakeFiles/high_state_ldt.dir/example/user/high_state_ldt.cpp.i

CMakeFiles/high_state_ldt.dir/example/user/high_state_ldt.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/high_state_ldt.dir/example/user/high_state_ldt.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/parallels/Projects/unitree_sdk2_ldt/example/user/high_state_ldt.cpp -o CMakeFiles/high_state_ldt.dir/example/user/high_state_ldt.cpp.s

# Object files for target high_state_ldt
high_state_ldt_OBJECTS = \
"CMakeFiles/high_state_ldt.dir/example/user/high_state_ldt.cpp.o"

# External object files for target high_state_ldt
high_state_ldt_EXTERNAL_OBJECTS =

high_state_ldt: CMakeFiles/high_state_ldt.dir/example/user/high_state_ldt.cpp.o
high_state_ldt: CMakeFiles/high_state_ldt.dir/build.make
high_state_ldt: CMakeFiles/high_state_ldt.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/parallels/Projects/unitree_sdk2_ldt/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable high_state_ldt"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/high_state_ldt.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/high_state_ldt.dir/build: high_state_ldt

.PHONY : CMakeFiles/high_state_ldt.dir/build

CMakeFiles/high_state_ldt.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/high_state_ldt.dir/cmake_clean.cmake
.PHONY : CMakeFiles/high_state_ldt.dir/clean

CMakeFiles/high_state_ldt.dir/depend:
	cd /home/parallels/Projects/unitree_sdk2_ldt/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/parallels/Projects/unitree_sdk2_ldt /home/parallels/Projects/unitree_sdk2_ldt /home/parallels/Projects/unitree_sdk2_ldt/build /home/parallels/Projects/unitree_sdk2_ldt/build /home/parallels/Projects/unitree_sdk2_ldt/build/CMakeFiles/high_state_ldt.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/high_state_ldt.dir/depend

