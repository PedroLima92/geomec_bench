# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.13

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
CMAKE_COMMAND = /Applications/CMake.app/Contents/bin/cmake

# The command to remove a file.
RM = /Applications/CMake.app/Contents/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /Users/pedro/projetos/geomec_bench

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /Users/pedro/projetos/geomec_bench/build

# Include any dependencies generated for this target.
include MHM_Frac/CMakeFiles/FractureIntersection.dir/depend.make

# Include the progress variables for this target.
include MHM_Frac/CMakeFiles/FractureIntersection.dir/progress.make

# Include the compile flags for this target's objects.
include MHM_Frac/CMakeFiles/FractureIntersection.dir/flags.make

MHM_Frac/CMakeFiles/FractureIntersection.dir/TPZFractureIntersection.cpp.o: MHM_Frac/CMakeFiles/FractureIntersection.dir/flags.make
MHM_Frac/CMakeFiles/FractureIntersection.dir/TPZFractureIntersection.cpp.o: ../MHM_Frac/TPZFractureIntersection.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/pedro/projetos/geomec_bench/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object MHM_Frac/CMakeFiles/FractureIntersection.dir/TPZFractureIntersection.cpp.o"
	cd /Users/pedro/projetos/geomec_bench/build/MHM_Frac && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/FractureIntersection.dir/TPZFractureIntersection.cpp.o -c /Users/pedro/projetos/geomec_bench/MHM_Frac/TPZFractureIntersection.cpp

MHM_Frac/CMakeFiles/FractureIntersection.dir/TPZFractureIntersection.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/FractureIntersection.dir/TPZFractureIntersection.cpp.i"
	cd /Users/pedro/projetos/geomec_bench/build/MHM_Frac && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/pedro/projetos/geomec_bench/MHM_Frac/TPZFractureIntersection.cpp > CMakeFiles/FractureIntersection.dir/TPZFractureIntersection.cpp.i

MHM_Frac/CMakeFiles/FractureIntersection.dir/TPZFractureIntersection.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/FractureIntersection.dir/TPZFractureIntersection.cpp.s"
	cd /Users/pedro/projetos/geomec_bench/build/MHM_Frac && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/pedro/projetos/geomec_bench/MHM_Frac/TPZFractureIntersection.cpp -o CMakeFiles/FractureIntersection.dir/TPZFractureIntersection.cpp.s

MHM_Frac/CMakeFiles/FractureIntersection.dir/TPZPointCloud.cpp.o: MHM_Frac/CMakeFiles/FractureIntersection.dir/flags.make
MHM_Frac/CMakeFiles/FractureIntersection.dir/TPZPointCloud.cpp.o: ../MHM_Frac/TPZPointCloud.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/pedro/projetos/geomec_bench/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object MHM_Frac/CMakeFiles/FractureIntersection.dir/TPZPointCloud.cpp.o"
	cd /Users/pedro/projetos/geomec_bench/build/MHM_Frac && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/FractureIntersection.dir/TPZPointCloud.cpp.o -c /Users/pedro/projetos/geomec_bench/MHM_Frac/TPZPointCloud.cpp

MHM_Frac/CMakeFiles/FractureIntersection.dir/TPZPointCloud.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/FractureIntersection.dir/TPZPointCloud.cpp.i"
	cd /Users/pedro/projetos/geomec_bench/build/MHM_Frac && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/pedro/projetos/geomec_bench/MHM_Frac/TPZPointCloud.cpp > CMakeFiles/FractureIntersection.dir/TPZPointCloud.cpp.i

MHM_Frac/CMakeFiles/FractureIntersection.dir/TPZPointCloud.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/FractureIntersection.dir/TPZPointCloud.cpp.s"
	cd /Users/pedro/projetos/geomec_bench/build/MHM_Frac && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/pedro/projetos/geomec_bench/MHM_Frac/TPZPointCloud.cpp -o CMakeFiles/FractureIntersection.dir/TPZPointCloud.cpp.s

MHM_Frac/CMakeFiles/FractureIntersection.dir/FractureIntersectionConfig.cpp.o: MHM_Frac/CMakeFiles/FractureIntersection.dir/flags.make
MHM_Frac/CMakeFiles/FractureIntersection.dir/FractureIntersectionConfig.cpp.o: ../MHM_Frac/FractureIntersectionConfig.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/pedro/projetos/geomec_bench/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object MHM_Frac/CMakeFiles/FractureIntersection.dir/FractureIntersectionConfig.cpp.o"
	cd /Users/pedro/projetos/geomec_bench/build/MHM_Frac && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/FractureIntersection.dir/FractureIntersectionConfig.cpp.o -c /Users/pedro/projetos/geomec_bench/MHM_Frac/FractureIntersectionConfig.cpp

MHM_Frac/CMakeFiles/FractureIntersection.dir/FractureIntersectionConfig.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/FractureIntersection.dir/FractureIntersectionConfig.cpp.i"
	cd /Users/pedro/projetos/geomec_bench/build/MHM_Frac && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/pedro/projetos/geomec_bench/MHM_Frac/FractureIntersectionConfig.cpp > CMakeFiles/FractureIntersection.dir/FractureIntersectionConfig.cpp.i

MHM_Frac/CMakeFiles/FractureIntersection.dir/FractureIntersectionConfig.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/FractureIntersection.dir/FractureIntersectionConfig.cpp.s"
	cd /Users/pedro/projetos/geomec_bench/build/MHM_Frac && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/pedro/projetos/geomec_bench/MHM_Frac/FractureIntersectionConfig.cpp -o CMakeFiles/FractureIntersection.dir/FractureIntersectionConfig.cpp.s

MHM_Frac/CMakeFiles/FractureIntersection.dir/TPZElementIntersect.cpp.o: MHM_Frac/CMakeFiles/FractureIntersection.dir/flags.make
MHM_Frac/CMakeFiles/FractureIntersection.dir/TPZElementIntersect.cpp.o: ../MHM_Frac/TPZElementIntersect.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/pedro/projetos/geomec_bench/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object MHM_Frac/CMakeFiles/FractureIntersection.dir/TPZElementIntersect.cpp.o"
	cd /Users/pedro/projetos/geomec_bench/build/MHM_Frac && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/FractureIntersection.dir/TPZElementIntersect.cpp.o -c /Users/pedro/projetos/geomec_bench/MHM_Frac/TPZElementIntersect.cpp

MHM_Frac/CMakeFiles/FractureIntersection.dir/TPZElementIntersect.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/FractureIntersection.dir/TPZElementIntersect.cpp.i"
	cd /Users/pedro/projetos/geomec_bench/build/MHM_Frac && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/pedro/projetos/geomec_bench/MHM_Frac/TPZElementIntersect.cpp > CMakeFiles/FractureIntersection.dir/TPZElementIntersect.cpp.i

MHM_Frac/CMakeFiles/FractureIntersection.dir/TPZElementIntersect.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/FractureIntersection.dir/TPZElementIntersect.cpp.s"
	cd /Users/pedro/projetos/geomec_bench/build/MHM_Frac && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/pedro/projetos/geomec_bench/MHM_Frac/TPZElementIntersect.cpp -o CMakeFiles/FractureIntersection.dir/TPZElementIntersect.cpp.s

MHM_Frac/CMakeFiles/FractureIntersection.dir/VTKIntersect.cpp.o: MHM_Frac/CMakeFiles/FractureIntersection.dir/flags.make
MHM_Frac/CMakeFiles/FractureIntersection.dir/VTKIntersect.cpp.o: ../MHM_Frac/VTKIntersect.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/pedro/projetos/geomec_bench/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building CXX object MHM_Frac/CMakeFiles/FractureIntersection.dir/VTKIntersect.cpp.o"
	cd /Users/pedro/projetos/geomec_bench/build/MHM_Frac && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/FractureIntersection.dir/VTKIntersect.cpp.o -c /Users/pedro/projetos/geomec_bench/MHM_Frac/VTKIntersect.cpp

MHM_Frac/CMakeFiles/FractureIntersection.dir/VTKIntersect.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/FractureIntersection.dir/VTKIntersect.cpp.i"
	cd /Users/pedro/projetos/geomec_bench/build/MHM_Frac && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/pedro/projetos/geomec_bench/MHM_Frac/VTKIntersect.cpp > CMakeFiles/FractureIntersection.dir/VTKIntersect.cpp.i

MHM_Frac/CMakeFiles/FractureIntersection.dir/VTKIntersect.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/FractureIntersection.dir/VTKIntersect.cpp.s"
	cd /Users/pedro/projetos/geomec_bench/build/MHM_Frac && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/pedro/projetos/geomec_bench/MHM_Frac/VTKIntersect.cpp -o CMakeFiles/FractureIntersection.dir/VTKIntersect.cpp.s

# Object files for target FractureIntersection
FractureIntersection_OBJECTS = \
"CMakeFiles/FractureIntersection.dir/TPZFractureIntersection.cpp.o" \
"CMakeFiles/FractureIntersection.dir/TPZPointCloud.cpp.o" \
"CMakeFiles/FractureIntersection.dir/FractureIntersectionConfig.cpp.o" \
"CMakeFiles/FractureIntersection.dir/TPZElementIntersect.cpp.o" \
"CMakeFiles/FractureIntersection.dir/VTKIntersect.cpp.o"

# External object files for target FractureIntersection
FractureIntersection_EXTERNAL_OBJECTS =

MHM_Frac/FractureIntersection: MHM_Frac/CMakeFiles/FractureIntersection.dir/TPZFractureIntersection.cpp.o
MHM_Frac/FractureIntersection: MHM_Frac/CMakeFiles/FractureIntersection.dir/TPZPointCloud.cpp.o
MHM_Frac/FractureIntersection: MHM_Frac/CMakeFiles/FractureIntersection.dir/FractureIntersectionConfig.cpp.o
MHM_Frac/FractureIntersection: MHM_Frac/CMakeFiles/FractureIntersection.dir/TPZElementIntersect.cpp.o
MHM_Frac/FractureIntersection: MHM_Frac/CMakeFiles/FractureIntersection.dir/VTKIntersect.cpp.o
MHM_Frac/FractureIntersection: MHM_Frac/CMakeFiles/FractureIntersection.dir/build.make
MHM_Frac/FractureIntersection: /usr/local/lib/libmpfr.dylib
MHM_Frac/FractureIntersection: /usr/local/lib/libgmp.dylib
MHM_Frac/FractureIntersection: /usr/local/lib/libCGAL_Core.13.0.2.dylib
MHM_Frac/FractureIntersection: /usr/local/lib/libCGAL.13.0.2.dylib
MHM_Frac/FractureIntersection: /usr/local/lib/libboost_thread.dylib
MHM_Frac/FractureIntersection: /usr/local/lib/libboost_system.dylib
MHM_Frac/FractureIntersection: /usr/local/lib/libboost_chrono.dylib
MHM_Frac/FractureIntersection: /usr/local/lib/libboost_date_time.dylib
MHM_Frac/FractureIntersection: /usr/local/lib/libboost_atomic.dylib
MHM_Frac/FractureIntersection: /Users/pedro/projetos/neopz-install/pzlib/lib/libpz.a
MHM_Frac/FractureIntersection: /usr/local/lib/libmpfr.dylib
MHM_Frac/FractureIntersection: /usr/local/lib/libgmp.dylib
MHM_Frac/FractureIntersection: /usr/local/lib/libboost_thread.dylib
MHM_Frac/FractureIntersection: /usr/local/lib/libboost_chrono.dylib
MHM_Frac/FractureIntersection: /usr/local/lib/libboost_system.dylib
MHM_Frac/FractureIntersection: /usr/local/lib/libboost_date_time.dylib
MHM_Frac/FractureIntersection: /usr/local/lib/libboost_atomic.dylib
MHM_Frac/FractureIntersection: /usr/lib/libpthread.dylib
MHM_Frac/FractureIntersection: MHM_Frac/CMakeFiles/FractureIntersection.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/Users/pedro/projetos/geomec_bench/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Linking CXX executable FractureIntersection"
	cd /Users/pedro/projetos/geomec_bench/build/MHM_Frac && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/FractureIntersection.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
MHM_Frac/CMakeFiles/FractureIntersection.dir/build: MHM_Frac/FractureIntersection

.PHONY : MHM_Frac/CMakeFiles/FractureIntersection.dir/build

MHM_Frac/CMakeFiles/FractureIntersection.dir/clean:
	cd /Users/pedro/projetos/geomec_bench/build/MHM_Frac && $(CMAKE_COMMAND) -P CMakeFiles/FractureIntersection.dir/cmake_clean.cmake
.PHONY : MHM_Frac/CMakeFiles/FractureIntersection.dir/clean

MHM_Frac/CMakeFiles/FractureIntersection.dir/depend:
	cd /Users/pedro/projetos/geomec_bench/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /Users/pedro/projetos/geomec_bench /Users/pedro/projetos/geomec_bench/MHM_Frac /Users/pedro/projetos/geomec_bench/build /Users/pedro/projetos/geomec_bench/build/MHM_Frac /Users/pedro/projetos/geomec_bench/build/MHM_Frac/CMakeFiles/FractureIntersection.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : MHM_Frac/CMakeFiles/FractureIntersection.dir/depend

