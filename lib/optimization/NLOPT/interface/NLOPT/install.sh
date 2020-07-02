#!/bin/sh
# download, configure and build NLOPT mex file with MinGW (in a Msys2 console)
# configuration
# version of NLOPT to build
nloptversion='2.6.1'
# architecture to build (32 or 64)
arch=64
# make OPTI MEX interface with gnumex
makeOPTIMEXGnumex=true
# path to current file directory
basedir=$(cd `dirname $0` && pwd)
repositoryurlgit="https://github.com/stevengj/nlopt.git"
repositoryurlOPTIgit="https://github.com/jonathancurrie/OPTI.git"
MATLABhome64=/C/Progra~1/MATLAB/R2015b
MATLABhome32=/C/Progra~2/MATLAB/R2015b

## beginning of script
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'
MEXsuffix="mexw64"
rootdir="$basedir/NLOPT_$nloptversion"
if [ "$arch" -eq 32 ]; then
	rootdir="${rootdir}_32bit"
	MEXsuffix="mexw32"
fi
rootdir="${rootdir}/"
cd "$basedir"

majorversion="$(cut -d'.' -f1 <<<$nloptversion)"
minorversion="$(cut -d'.' -f2 <<<$nloptversion)"
bugfixversion="$(cut -d'.' -f3 <<<$nloptversion)"
# get requested NLOPT version and checkout/clone into working directory
hasgit=0
if [ "$majorversion" -ge 2 -a "$minorversion" -ge 6 ]; then
	nloptbranch="v${nloptversion}"
else
	nloptbranch="nlopt-${nloptversion}"
fi
if [ ! -d "$rootdir" ]; then
	mkdir "$rootdir"
	if [ $? -gt 0 ]; then
		echo "Could not create directory for NLOPT"
		exit 1
	fi
	curl -s --head "$repositoryurlgit" | head -n 1 | grep "HTTP/1.[01] [23].." > /dev/null
	hasgit=$?
	if [ "$hasgit" -eq 0 ]; then
		git clone -b "$nloptbranch" --depth 1 --recurse-submodules "$repositoryurlgit" "$rootdir"
		if [ $? -gt 0 ]; then
			echo -e "${RED}Could not clone NLOPT version ${nloptversion} from '$repositoryurlgit' in branch '$nloptbranch'${NC}"
			exit 1
		fi
	else
		echo -e "${RED}NLOPT repository for version ${nloptversion} is not reachable${NC}"
		exit 1
	fi
fi

directoryBuild="$rootdir/build"
cd "$rootdir"
if [ ! -d "$directoryBuild" ]; then
	mkdir "$directoryBuild"
	if [ $? -gt 0 ]; then
		echo -e "${RED}Could not create build directory for NLOPT${NC}"
		exit 1
	fi
fi
directoryLIB="${directoryBuild}/lib"
if [ ! -d "$directoryLIB" ]; then
	mkdir "$directoryLIB"
	if [ $? -gt 0 ]; then
		echo -e "${RED}Could not create build directory for NLOPT${NC}"
		exit 1
	fi
fi
directoryMEX="${directoryBuild}/matlab"
if [ ! -d "$directoryMEX" ]; then
	mkdir "$directoryMEX"
	if [ $? -gt 0 ]; then
		echo -e "${RED}Could not create build directory for NLOPT${NC}"
		exit 1
	fi
fi
directoryOCT="${directoryBuild}/octave"
if [ ! -d "$directoryOCT" ]; then
	mkdir "$directoryOCT"
	if [ $? -gt 0 ]; then
		echo -e "${RED}Could not create build directory for NLOPT${NC}"
		exit 1
	fi
fi
cd "$directoryBuild"

if [ "$arch" -eq 32 ]; then
	echo -e "${YELLOW}32 bit compilation is currently not supported and may generate 64 bit MEX files as well${NC}"
	MATLABhome=$MATLABhome32
else
	MATLABhome=$MATLABhome64
fi
# set options for static linking
if [ "$1" == "-static" ]; then
	static=" -DBUILD_SHARED_LIBS=OFF"
	# on windows libstdc and libstc++ are linked statically
	git apply "$rootdir/../CMakeLists.txt.patch"
	if [ $? -gt 0 ]; then
		echo -e "${RED}Could not apply patch for static linking of libstc(++)${NC}"
		exit 1
	fi
else
	static=""
fi
# start build process
cmake -G"MSYS Makefiles" $static -DMatlab_ROOT_DIR=$MATLABhome -DCMAKE_INSTALL_PREFIX=$directoryLIB -DINSTALL_MEX_DIR=$directoryMEX -DINSTALL_OCT_DIR=$directoryOCT -DINSTALL_M_DIR=$directoryOCT ".."
if [ $? -gt 0 ]; then
	echo -e "${RED}Could not configure NLOPT${NC}"
	exit -2
fi
make -j3
if [ $? -gt 0 ]; then
	echo -e "${RED}Could not make NLOPT${NC}"
	exit -2
fi
make install
if [ $? -gt 0 ]; then
	echo -e "${RED}Could not install NLOPT${NC}"
	exit -2
fi

# build OPTI interface
directoryOPTI="$rootdir/OPTI"
if [ ! -d "$directoryOPTI" ]; then
	mkdir "$directoryOPTI"
	if [ $? -gt 0 ]; then
		echo -e "${RED}Could not create directory for NLOPT OPTI interface${NC}"
		exit 1
	fi
	curl -s --head "$repositoryurlOPTIgit" | head -n 1 | grep "HTTP/1.[01] [23].." > /dev/null
	hasgit=$?
	if [ "$hasgit" -eq 0 ]; then
		git clone -b "master" --depth 1 --recurse-submodules "$repositoryurlOPTIgit" "$directoryOPTI"
		if [ $? -gt 0 ]; then
			echo -e "${RED}Could not clone NLOPT OPTI interface from '$repositoryurlOPTIgit' in branch 'master'${NC}"
			exit 1
		fi
	else
		echo -e "${RED}NLOPT OPTI interface repository is not reachable${NC}"
		exit 1
	fi
	# patch opti_mex_utils.cpp to work with C++
	cd "$directoryOPTI"
	git apply "$rootdir/../../opti_mex_utils.cpp.patch"
	if [ $? -gt 0 ]; then
		echo -e "${RED}Could not apply patch for opti_mex_utils.cpp${NC}"
		exit 1
	fi
	cd "$directoryBuild"
fi

echo -e "${GREEN}build OPTI MEX interface${NC}"
directoryBuildwindows=${directoryBuild#"/D"}
directoryBuildwindows="D:${directoryBuildwindows}"
directoryLIBwindows=${directoryLIB#"/D"}
directoryLIBwindows="D:${directoryLIBwindows}"
directoryOPTIwindows=${directoryOPTI#"/D"}
directoryOPTIwindows="D:${directoryOPTIwindows}"
mexINCLUDEstring="-I${directoryBuildwindows}/lib/include -IInclude/Nlopt -Inlopt/Include -I${directoryOPTIwindows}/Solvers/Source/opti -I${directoryOPTIwindows}/Solvers/Source/nlopt"
if [ ! "$makeOPTIMEXGnumex" == "true" ]; then
	mexSRCstring="${directoryOPTIwindows}/Solvers/Source/nloptmex.c ${directoryOPTIwindows}/Solvers/Source/opti/opti_mex_utils.cpp"
	mexCXX_OPTS='CXXFLAGS='\''$CXXFLAGS -fpermissive -std=c++11'\'''
	mexCOMP_OPTS='LDFLAGS='\''$LDFLAGS -fpermissive -std=c++11'\'''
	mexLIBstring="${directoryBuildwindows}/lib/lib/libnlopt.a ${directoryBuildwindows}/matlab/libnlopt_optimize.dll.a -L${directoryLIBwindows} ${directoryBuildwindows}/src/octave/CMakeFiles/nlopt_optimize-mex.dir/objects.a"
	mexbuildstring="mex -v -largeArrayDims ${mexINCLUDEstring} ${mexLIBstring} -DML_VER=8.6 -DOPTI_VER=2.28 ${mexSRCstring} ${mexCOMP_OPTS} ${mexCXX_OPTS}"
	echo "$mexbuildstring"
	$MATLABhome/bin/matlab -r "${mexbuildstring};exit('force')"
	if [ $? -gt 0 ]; then
		echo -e "${RED}Could not make MEX interface${NC}"
		exit -1
	fi
	exit 0
else
	cd "$directoryOPTI"
	cp "$rootdir/../get.Gnumex" "$directoryMEX"
	cp "$rootdir/../Makefile.in" "$directoryMEX/Makefile"
	prefix="/mingw64"
	if [ "$arch" -eq 32 ]; then
		prefix="/mingw32"
	fi
	echo -e "${GREEN}preparing makefiles${NC}"
	sed -i "s,@MATLAB_HOME@,$MATLABhome," "$directoryMEX/Makefile"
	sed -i "s,@MEXSUFFIX@,$MEXsuffix," "$directoryMEX/Makefile"
	sed -i "s,@MEX_WINDOWS_FALSE@,#," "$directoryMEX/Makefile"
	sed -i "s,@MEX_WINDOWS_TRUE@,," "$directoryMEX/Makefile"
	sed -i "s,@exec_prefix@,\${prefix}," "$directoryMEX/Makefile"
	sed -i "s,@prefix@,$prefix," "$directoryMEX/Makefile"
	sed -i "s,@libdir@,\${exec_prefix}/lib," "$directoryMEX/Makefile"
	sed -i "s,@srcdir@,./src," "$directoryMEX/Makefile"
	sed -i "s,@abs_include_dir@,$prefix/lib," "$directoryMEX/Makefile"
	sed -i "s,@abs_lib_dir@,$prefix/include," "$directoryMEX/Makefile"
	sed -i "s,@CXX@,g++," "$directoryMEX/Makefile"
	sed -i "s,@F77@,gfortran," "$directoryMEX/Makefile"
	sed -i "s,@OBJEXT@,obj," "$directoryMEX/Makefile"
	sed -i "s,@CXXFLAGS@, -O3 -fpermissive -fopenmp -std=c++11 -DML_VER=8.6 -DOPTI_VER=2.28," "$directoryMEX/Makefile"
	sed -i "s,@MEXFLAGS@, -DML_VER=8.6 -DOPTI_VER=2.28," "$directoryMEX/Makefile"
	sed -i "s,@INCLUDE@,$mexINCLUDEstring," "$directoryMEX/Makefile"
	sed -i "s,@LIBS_STATIC_FILE@,-L$directoryMEX/gnumex/libdef -llibut ${directoryBuild}/lib/lib/libnlopt.a ${directoryBuild}/matlab/libnlopt_optimize.dll.a -L${directoryLIB} ${directoryBuild}/src/octave/CMakeFiles/nlopt_optimize-mex.dir/objects.a," "$directoryMEX/Makefile"
	PKG_CONFIG=pkg-config
	if test -n "$PKG_CONFIG"; then
		sed -i "s,@COIN_HAS_PKGCONFIG_TRUE@,," "$directoryMEX/Makefile"
		sed -i "s,@COIN_HAS_PKGCONFIG_FALSE@,#," "$directoryMEX/Makefile"
	else
		sed -i "s,@COIN_HAS_PKGCONFIG_TRUE@,#," "$directoryMEX/Makefile"
		sed -i "s,@COIN_HAS_PKGCONFIG_FALSE@,," "$directoryMEX/Makefile"
	fi
	COIN_PKG_CONFIG_PATH="$PKG_CONFIG_PATH"
	sed -i "s,@COIN_PKG_CONFIG_PATH@,${directoryBuild}:$COIN_PKG_CONFIG_PATH," "$directoryMEX/Makefile"
	sed -i "s,@PKG_CONFIG@,$PKG_CONFIG," "$directoryMEX/Makefile"
	sed -i "s,@IPOPTLIB_CFLAGS_INSTALLED@,," "$directoryMEX/Makefile"
	sed -i "s,@IPOPTLIB_LIBS_INSTALLED@,," "$directoryMEX/Makefile"
	sed -i "s,@COIN_CXX_IS_CL_TRUE@,," "$directoryMEX/Makefile"
	sed -i "s,@COIN_CXX_IS_CL_FALSE@,#," "$directoryMEX/Makefile"
	sed -i "s,@MATLAB_CYGPATH_W@,cygpath -w," "$directoryMEX/Makefile"
	if [ ! "$static" == "" ]; then
		sed -i "s,@MEX_STATIC_FALSE@,#," "$directoryMEX/Makefile"
		sed -i "s,@MEX_STATIC_TRUE@,," "$directoryMEX/Makefile"
	else
		sed -i "s,@MEX_STATIC_FALSE@,," "$directoryMEX/Makefile"
		sed -i "s,@MEX_STATIC_TRUE@,#," "$directoryMEX/Makefile"
	fi
	echo -e "${GREEN}getting GNUMEX${NC}"
	cd "$directoryMEX"
	sh get.Gnumex
	if [ $? -gt 0 ]; then
		echo -e "${RED}Could not get GNUMEX${NC}"
		exit -2
	fi
	# copy files from OPTI to MEX interface directory
	if [ ! -d "$directoryMEX/src/" ]; then
		mkdir "$directoryMEX/src/"
	fi
	cp "${directoryOPTI}/Solvers/Source/nloptmex.c" "$directoryMEX/src/"
	cp "${directoryOPTI}/Solvers/Source/opti/opti_mex_utils.cpp" "$directoryMEX/src/"
	sed -i "s|#define MAJOR_VERSION 2|#define MAJOR_VERSION $majorversion|" "${directoryOPTI}/Solvers/Source/nlopt/config.h"
	sed -i "s|#define MINOR_VERSION 4|#define MINOR_VERSION $minorversion|" "${directoryOPTI}/Solvers/Source/nlopt/config.h"
	sed -i "s|#define BUGFIX_VERSION 2|#define BUGFIX_VERSION $bugfixversion|" "${directoryOPTI}/Solvers/Source/nlopt/config.h"
	sed -i "s|#define PACKAGE_VERSION \"2.4.2\"|#define PACKAGE_VERSION \"$majorversion.$minorversion.$bugfixversion\"|" "${directoryOPTI}/Solvers/Source/nlopt/config.h"
	# build mex file
	cd "$directoryMEX"
	# libtool crashes when creating def files for 'microsoft' libraries
	sed -i 's/\\extern\\lib\\win64\\microsoft/\\extern\\lib\\win64\\mingw64/g' "$directoryMEX/gnumex/gnumex.m"
	make gnumex --always-make
	if [ $? -gt 0 ]; then
		echo -e "${RED}Could not configure MATLAB${NC}"
		exit -2
	fi
	# add libut to gnumex libdef path
	if [ "$arch" -eq 32 ]; then
		if [ ! -f "$MATLABhome/extern/lib/win32/lcc/libut.lib" ]; then
			echo -e "${RED}libut not found in MATLAB installation directory${NC}"
			exit 1
		fi
		cp "$MATLABhome/extern/lib/win32/lcc/libut.lib" "$directoryMEX/gnumex/libdef"
	else
		if [ ! -f "$MATLABhome/extern/lib/win64/mingw64/libut.lib" ]; then
			echo -e "${RED}libut not found in MATLAB installation directory${NC}"
			exit 1
		fi
		cp "$MATLABhome/extern/lib/win64/mingw64/libut.lib" "$directoryMEX/gnumex/libdef"
	fi
	echo "LIBRARY libut.dll" > "$directoryMEX/gnumex/libdef/libut.def"
	echo EXPORTS >> "$directoryMEX/gnumex/libdef/libut.def"
	nm "$directoryMEX/gnumex/libdef/libut.lib" | grep ' T ' | sed 's/.* T //' >> "$directoryMEX/gnumex/libdef/libut.def"
	if [ $? -gt 0 ]; then
		echo -e "${RED}Could not export def for libut${NC}"
		exit -2
	fi
	make mexopts
	if [ $? -gt 0 ]; then
		echo -e "${RED}Could not make mexopts${NC}"
		exit -2
	fi
	# compile static
	sed -i 's/\(set GM_ADD_LIBS=\)\(-llibmx -llibmex -llibmat -llibut.*\)/\1-static \2/' 'mexopts.bat'
	if [ $? -gt 0 ]; then
		echo -e "${RED}Could not edit mexopts${NC}"
		exit -2
	fi
	make install
	if [ $? -gt 0 ]; then
		echo -e "${RED}Could not make mex file${NC}"
		exit -2
	fi
	exit 0
fi
echo -e "${YELLOW}Copy nlopt.$MEXsuffix and nlopt_optimize.$MEXsuffix to a folder where Matlab can find it${NC}"
