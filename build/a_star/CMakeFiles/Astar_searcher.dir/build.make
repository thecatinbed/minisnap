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
CMAKE_SOURCE_DIR = /home/mm/catkin_ws/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/mm/catkin_ws/build

# Include any dependencies generated for this target.
include a_star/CMakeFiles/Astar_searcher.dir/depend.make

# Include the progress variables for this target.
include a_star/CMakeFiles/Astar_searcher.dir/progress.make

# Include the compile flags for this target's objects.
include a_star/CMakeFiles/Astar_searcher.dir/flags.make

a_star/CMakeFiles/Astar_searcher.dir/src/Astar_searcher.cpp.o: a_star/CMakeFiles/Astar_searcher.dir/flags.make
a_star/CMakeFiles/Astar_searcher.dir/src/Astar_searcher.cpp.o: /home/mm/catkin_ws/src/a_star/src/Astar_searcher.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/mm/catkin_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object a_star/CMakeFiles/Astar_searcher.dir/src/Astar_searcher.cpp.o"
	cd /home/mm/catkin_ws/build/a_star && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/Astar_searcher.dir/src/Astar_searcher.cpp.o -c /home/mm/catkin_ws/src/a_star/src/Astar_searcher.cpp

a_star/CMakeFiles/Astar_searcher.dir/src/Astar_searcher.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Astar_searcher.dir/src/Astar_searcher.cpp.i"
	cd /home/mm/catkin_ws/build/a_star && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/mm/catkin_ws/src/a_star/src/Astar_searcher.cpp > CMakeFiles/Astar_searcher.dir/src/Astar_searcher.cpp.i

a_star/CMakeFiles/Astar_searcher.dir/src/Astar_searcher.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Astar_searcher.dir/src/Astar_searcher.cpp.s"
	cd /home/mm/catkin_ws/build/a_star && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/mm/catkin_ws/src/a_star/src/Astar_searcher.cpp -o CMakeFiles/Astar_searcher.dir/src/Astar_searcher.cpp.s

# Object files for target Astar_searcher
Astar_searcher_OBJECTS = \
"CMakeFiles/Astar_searcher.dir/src/Astar_searcher.cpp.o"

# External object files for target Astar_searcher
Astar_searcher_EXTERNAL_OBJECTS =

/home/mm/catkin_ws/devel/lib/libAstar_searcher.so: a_star/CMakeFiles/Astar_searcher.dir/src/Astar_searcher.cpp.o
/home/mm/catkin_ws/devel/lib/libAstar_searcher.so: a_star/CMakeFiles/Astar_searcher.dir/build.make
/home/mm/catkin_ws/devel/lib/libAstar_searcher.so: a_star/CMakeFiles/Astar_searcher.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/mm/catkin_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX shared library /home/mm/catkin_ws/devel/lib/libAstar_searcher.so"
	cd /home/mm/catkin_ws/build/a_star && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/Astar_searcher.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
a_star/CMakeFiles/Astar_searcher.dir/build: /home/mm/catkin_ws/devel/lib/libAstar_searcher.so

.PHONY : a_star/CMakeFiles/Astar_searcher.dir/build

a_star/CMakeFiles/Astar_searcher.dir/clean:
	cd /home/mm/catkin_ws/build/a_star && $(CMAKE_COMMAND) -P CMakeFiles/Astar_searcher.dir/cmake_clean.cmake
.PHONY : a_star/CMakeFiles/Astar_searcher.dir/clean

a_star/CMakeFiles/Astar_searcher.dir/depend:
	cd /home/mm/catkin_ws/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/mm/catkin_ws/src /home/mm/catkin_ws/src/a_star /home/mm/catkin_ws/build /home/mm/catkin_ws/build/a_star /home/mm/catkin_ws/build/a_star/CMakeFiles/Astar_searcher.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : a_star/CMakeFiles/Astar_searcher.dir/depend

