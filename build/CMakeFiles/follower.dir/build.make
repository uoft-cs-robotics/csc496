# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.26

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
CMAKE_COMMAND = /opt/cmake-3.26.3-linux-x86_64/bin/cmake

# The command to remove a file.
RM = /opt/cmake-3.26.3-linux-x86_64/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/lueder/medcvr_git/csc496

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/lueder/medcvr_git/csc496/build

# Include any dependencies generated for this target.
include CMakeFiles/follower.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/follower.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/follower.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/follower.dir/flags.make

CMakeFiles/follower.dir/src/teleop/follower.cpp.o: CMakeFiles/follower.dir/flags.make
CMakeFiles/follower.dir/src/teleop/follower.cpp.o: /home/lueder/medcvr_git/csc496/src/teleop/follower.cpp
CMakeFiles/follower.dir/src/teleop/follower.cpp.o: CMakeFiles/follower.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/lueder/medcvr_git/csc496/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/follower.dir/src/teleop/follower.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/follower.dir/src/teleop/follower.cpp.o -MF CMakeFiles/follower.dir/src/teleop/follower.cpp.o.d -o CMakeFiles/follower.dir/src/teleop/follower.cpp.o -c /home/lueder/medcvr_git/csc496/src/teleop/follower.cpp

CMakeFiles/follower.dir/src/teleop/follower.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/follower.dir/src/teleop/follower.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/lueder/medcvr_git/csc496/src/teleop/follower.cpp > CMakeFiles/follower.dir/src/teleop/follower.cpp.i

CMakeFiles/follower.dir/src/teleop/follower.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/follower.dir/src/teleop/follower.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/lueder/medcvr_git/csc496/src/teleop/follower.cpp -o CMakeFiles/follower.dir/src/teleop/follower.cpp.s

# Object files for target follower
follower_OBJECTS = \
"CMakeFiles/follower.dir/src/teleop/follower.cpp.o"

# External object files for target follower
follower_EXTERNAL_OBJECTS =

libfollower.a: CMakeFiles/follower.dir/src/teleop/follower.cpp.o
libfollower.a: CMakeFiles/follower.dir/build.make
libfollower.a: CMakeFiles/follower.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/lueder/medcvr_git/csc496/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX static library libfollower.a"
	$(CMAKE_COMMAND) -P CMakeFiles/follower.dir/cmake_clean_target.cmake
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/follower.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/follower.dir/build: libfollower.a
.PHONY : CMakeFiles/follower.dir/build

CMakeFiles/follower.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/follower.dir/cmake_clean.cmake
.PHONY : CMakeFiles/follower.dir/clean

CMakeFiles/follower.dir/depend:
	cd /home/lueder/medcvr_git/csc496/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/lueder/medcvr_git/csc496 /home/lueder/medcvr_git/csc496 /home/lueder/medcvr_git/csc496/build /home/lueder/medcvr_git/csc496/build /home/lueder/medcvr_git/csc496/build/CMakeFiles/follower.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/follower.dir/depend

