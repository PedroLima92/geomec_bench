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
include MHM_Frac/CMakeFiles/FracFlowGmsh.dir/depend.make

# Include the progress variables for this target.
include MHM_Frac/CMakeFiles/FracFlowGmsh.dir/progress.make

# Include the compile flags for this target's objects.
include MHM_Frac/CMakeFiles/FracFlowGmsh.dir/flags.make

MHM_Frac/CMakeFiles/FracFlowGmsh.dir/FracFlowGmsh.cpp.o: MHM_Frac/CMakeFiles/FracFlowGmsh.dir/flags.make
MHM_Frac/CMakeFiles/FracFlowGmsh.dir/FracFlowGmsh.cpp.o: ../MHM_Frac/FracFlowGmsh.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/pedro/projetos/geomec_bench/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object MHM_Frac/CMakeFiles/FracFlowGmsh.dir/FracFlowGmsh.cpp.o"
	cd /Users/pedro/projetos/geomec_bench/build/MHM_Frac && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/FracFlowGmsh.dir/FracFlowGmsh.cpp.o -c /Users/pedro/projetos/geomec_bench/MHM_Frac/FracFlowGmsh.cpp

MHM_Frac/CMakeFiles/FracFlowGmsh.dir/FracFlowGmsh.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/FracFlowGmsh.dir/FracFlowGmsh.cpp.i"
	cd /Users/pedro/projetos/geomec_bench/build/MHM_Frac && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/pedro/projetos/geomec_bench/MHM_Frac/FracFlowGmsh.cpp > CMakeFiles/FracFlowGmsh.dir/FracFlowGmsh.cpp.i

MHM_Frac/CMakeFiles/FracFlowGmsh.dir/FracFlowGmsh.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/FracFlowGmsh.dir/FracFlowGmsh.cpp.s"
	cd /Users/pedro/projetos/geomec_bench/build/MHM_Frac && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/pedro/projetos/geomec_bench/MHM_Frac/FracFlowGmsh.cpp -o CMakeFiles/FracFlowGmsh.dir/FracFlowGmsh.cpp.s

MHM_Frac/CMakeFiles/FracFlowGmsh.dir/TPZMatLaplacianHybrid.cpp.o: MHM_Frac/CMakeFiles/FracFlowGmsh.dir/flags.make
MHM_Frac/CMakeFiles/FracFlowGmsh.dir/TPZMatLaplacianHybrid.cpp.o: ../MHM_Frac/TPZMatLaplacianHybrid.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/pedro/projetos/geomec_bench/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object MHM_Frac/CMakeFiles/FracFlowGmsh.dir/TPZMatLaplacianHybrid.cpp.o"
	cd /Users/pedro/projetos/geomec_bench/build/MHM_Frac && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/FracFlowGmsh.dir/TPZMatLaplacianHybrid.cpp.o -c /Users/pedro/projetos/geomec_bench/MHM_Frac/TPZMatLaplacianHybrid.cpp

MHM_Frac/CMakeFiles/FracFlowGmsh.dir/TPZMatLaplacianHybrid.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/FracFlowGmsh.dir/TPZMatLaplacianHybrid.cpp.i"
	cd /Users/pedro/projetos/geomec_bench/build/MHM_Frac && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/pedro/projetos/geomec_bench/MHM_Frac/TPZMatLaplacianHybrid.cpp > CMakeFiles/FracFlowGmsh.dir/TPZMatLaplacianHybrid.cpp.i

MHM_Frac/CMakeFiles/FracFlowGmsh.dir/TPZMatLaplacianHybrid.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/FracFlowGmsh.dir/TPZMatLaplacianHybrid.cpp.s"
	cd /Users/pedro/projetos/geomec_bench/build/MHM_Frac && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/pedro/projetos/geomec_bench/MHM_Frac/TPZMatLaplacianHybrid.cpp -o CMakeFiles/FracFlowGmsh.dir/TPZMatLaplacianHybrid.cpp.s

MHM_Frac/CMakeFiles/FracFlowGmsh.dir/TPZFracSet.cpp.o: MHM_Frac/CMakeFiles/FracFlowGmsh.dir/flags.make
MHM_Frac/CMakeFiles/FracFlowGmsh.dir/TPZFracSet.cpp.o: ../MHM_Frac/TPZFracSet.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/pedro/projetos/geomec_bench/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object MHM_Frac/CMakeFiles/FracFlowGmsh.dir/TPZFracSet.cpp.o"
	cd /Users/pedro/projetos/geomec_bench/build/MHM_Frac && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/FracFlowGmsh.dir/TPZFracSet.cpp.o -c /Users/pedro/projetos/geomec_bench/MHM_Frac/TPZFracSet.cpp

MHM_Frac/CMakeFiles/FracFlowGmsh.dir/TPZFracSet.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/FracFlowGmsh.dir/TPZFracSet.cpp.i"
	cd /Users/pedro/projetos/geomec_bench/build/MHM_Frac && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/pedro/projetos/geomec_bench/MHM_Frac/TPZFracSet.cpp > CMakeFiles/FracFlowGmsh.dir/TPZFracSet.cpp.i

MHM_Frac/CMakeFiles/FracFlowGmsh.dir/TPZFracSet.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/FracFlowGmsh.dir/TPZFracSet.cpp.s"
	cd /Users/pedro/projetos/geomec_bench/build/MHM_Frac && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/pedro/projetos/geomec_bench/MHM_Frac/TPZFracSet.cpp -o CMakeFiles/FracFlowGmsh.dir/TPZFracSet.cpp.s

MHM_Frac/CMakeFiles/FracFlowGmsh.dir/TPZFracSimulation.cpp.o: MHM_Frac/CMakeFiles/FracFlowGmsh.dir/flags.make
MHM_Frac/CMakeFiles/FracFlowGmsh.dir/TPZFracSimulation.cpp.o: ../MHM_Frac/TPZFracSimulation.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/pedro/projetos/geomec_bench/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object MHM_Frac/CMakeFiles/FracFlowGmsh.dir/TPZFracSimulation.cpp.o"
	cd /Users/pedro/projetos/geomec_bench/build/MHM_Frac && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/FracFlowGmsh.dir/TPZFracSimulation.cpp.o -c /Users/pedro/projetos/geomec_bench/MHM_Frac/TPZFracSimulation.cpp

MHM_Frac/CMakeFiles/FracFlowGmsh.dir/TPZFracSimulation.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/FracFlowGmsh.dir/TPZFracSimulation.cpp.i"
	cd /Users/pedro/projetos/geomec_bench/build/MHM_Frac && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/pedro/projetos/geomec_bench/MHM_Frac/TPZFracSimulation.cpp > CMakeFiles/FracFlowGmsh.dir/TPZFracSimulation.cpp.i

MHM_Frac/CMakeFiles/FracFlowGmsh.dir/TPZFracSimulation.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/FracFlowGmsh.dir/TPZFracSimulation.cpp.s"
	cd /Users/pedro/projetos/geomec_bench/build/MHM_Frac && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/pedro/projetos/geomec_bench/MHM_Frac/TPZFracSimulation.cpp -o CMakeFiles/FracFlowGmsh.dir/TPZFracSimulation.cpp.s

# Object files for target FracFlowGmsh
FracFlowGmsh_OBJECTS = \
"CMakeFiles/FracFlowGmsh.dir/FracFlowGmsh.cpp.o" \
"CMakeFiles/FracFlowGmsh.dir/TPZMatLaplacianHybrid.cpp.o" \
"CMakeFiles/FracFlowGmsh.dir/TPZFracSet.cpp.o" \
"CMakeFiles/FracFlowGmsh.dir/TPZFracSimulation.cpp.o"

# External object files for target FracFlowGmsh
FracFlowGmsh_EXTERNAL_OBJECTS =

MHM_Frac/FracFlowGmsh: MHM_Frac/CMakeFiles/FracFlowGmsh.dir/FracFlowGmsh.cpp.o
MHM_Frac/FracFlowGmsh: MHM_Frac/CMakeFiles/FracFlowGmsh.dir/TPZMatLaplacianHybrid.cpp.o
MHM_Frac/FracFlowGmsh: MHM_Frac/CMakeFiles/FracFlowGmsh.dir/TPZFracSet.cpp.o
MHM_Frac/FracFlowGmsh: MHM_Frac/CMakeFiles/FracFlowGmsh.dir/TPZFracSimulation.cpp.o
MHM_Frac/FracFlowGmsh: MHM_Frac/CMakeFiles/FracFlowGmsh.dir/build.make
MHM_Frac/FracFlowGmsh: /usr/local/lib/libmpfr.dylib
MHM_Frac/FracFlowGmsh: /usr/local/lib/libgmp.dylib
MHM_Frac/FracFlowGmsh: /usr/local/lib/libCGAL_Core.13.0.2.dylib
MHM_Frac/FracFlowGmsh: /usr/local/lib/libCGAL.13.0.2.dylib
MHM_Frac/FracFlowGmsh: /usr/local/lib/libboost_thread.dylib
MHM_Frac/FracFlowGmsh: /usr/local/lib/libboost_system.dylib
MHM_Frac/FracFlowGmsh: /usr/local/lib/libboost_chrono.dylib
MHM_Frac/FracFlowGmsh: /usr/local/lib/libboost_date_time.dylib
MHM_Frac/FracFlowGmsh: /usr/local/lib/libboost_atomic.dylib
MHM_Frac/FracFlowGmsh: /Users/pedro/projetos/neopz-install/pzlib/lib/libpz.a
MHM_Frac/FracFlowGmsh: /usr/local/lib/libmpfr.dylib
MHM_Frac/FracFlowGmsh: /usr/local/lib/libgmp.dylib
MHM_Frac/FracFlowGmsh: /usr/local/lib/libboost_thread.dylib
MHM_Frac/FracFlowGmsh: /usr/local/lib/libboost_chrono.dylib
MHM_Frac/FracFlowGmsh: /usr/local/lib/libboost_system.dylib
MHM_Frac/FracFlowGmsh: /usr/local/lib/libboost_date_time.dylib
MHM_Frac/FracFlowGmsh: /usr/local/lib/libboost_atomic.dylib
MHM_Frac/FracFlowGmsh: /usr/lib/libpthread.dylib
MHM_Frac/FracFlowGmsh: MHM_Frac/CMakeFiles/FracFlowGmsh.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/Users/pedro/projetos/geomec_bench/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Linking CXX executable FracFlowGmsh"
	cd /Users/pedro/projetos/geomec_bench/build/MHM_Frac && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/FracFlowGmsh.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
MHM_Frac/CMakeFiles/FracFlowGmsh.dir/build: MHM_Frac/FracFlowGmsh

.PHONY : MHM_Frac/CMakeFiles/FracFlowGmsh.dir/build

MHM_Frac/CMakeFiles/FracFlowGmsh.dir/clean:
	cd /Users/pedro/projetos/geomec_bench/build/MHM_Frac && $(CMAKE_COMMAND) -P CMakeFiles/FracFlowGmsh.dir/cmake_clean.cmake
.PHONY : MHM_Frac/CMakeFiles/FracFlowGmsh.dir/clean

MHM_Frac/CMakeFiles/FracFlowGmsh.dir/depend:
	cd /Users/pedro/projetos/geomec_bench/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /Users/pedro/projetos/geomec_bench /Users/pedro/projetos/geomec_bench/MHM_Frac /Users/pedro/projetos/geomec_bench/build /Users/pedro/projetos/geomec_bench/build/MHM_Frac /Users/pedro/projetos/geomec_bench/build/MHM_Frac/CMakeFiles/FracFlowGmsh.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : MHM_Frac/CMakeFiles/FracFlowGmsh.dir/depend

