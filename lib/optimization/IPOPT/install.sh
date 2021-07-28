#!/bin/bash
# download, configure and build IPOPT mex file with MinGW (in a Msys2 console)
# configuration
# version of IPOPT to build
ipoptversion='3.12.13'
# architecture to build (32 or 64)
arch=64
# path to current file directory
basedir=$(cd "$(dirname "$0")" && pwd)
# makeGNUMEX=true
repositoryurlsvn="https://projects.coin-or.org/svn/Ipopt/releases/$ipoptversion/"
repositoryurlgit="https://github.com/coin-or/Ipopt"
repositoryurlUtils="https://github.com/coin-or/CoinUtils"
repositoryurlBLAS="https://github.com/coin-or-tools/ThirdParty-Blas"
repositoryurlLAPACK="https://github.com/coin-or-tools/ThirdParty-Lapack"
repositoryurlASL="https://github.com/coin-or-tools/ThirdParty-ASL.git"
repositoryurlHSL="https://github.com/coin-or-tools/ThirdParty-HSL.git"
repositoryurlMUMPS="https://github.com/coin-or-tools/ThirdParty-Mumps.git"
repositoryurlMETIS="https://github.com/coin-or-tools/ThirdParty-Metis.git"
repositoryurlMEX="https://github.com/ebertolazzi/mexIPOPT"
MATLABhome64=/C/Progra~1/MATLAB/R2015b
MATLABhome32=/C/Progra~2/MATLAB/R2015b

## beginning of script
rootdir="$basedir/CoinIpopt_$ipoptversion"
if [ "$arch" -eq 32 ]; then
	rootdir="${rootdir}_32bit"
fi
rootdir="${rootdir}/"
# version that needs the patch 'configure.patch'
# shellcheck disable=SC2034
ipoptpatchneeded='3.12.10'
cd "$basedir" || (echo "Could not change directory" && exit 10)
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

# get requested IPOPT version and checkout/clone into working directory
hasgit=0
hassvn=0
if [ ! -d "$rootdir" ]; then
	mkdir "$rootdir"
	if [ $? -gt 0 ]; then
		echo -e "${RED}Could not create directory for IPOPT${NC}"
		exit 1
	fi
	curl -s --head "$repositoryurlsvn" | head -n 1 | grep "HTTP/1.[01] [23].." > /dev/null
	hassvn=$?
	if [ ! "$hassvn" -eq 0 ]; then
		curl -s --head "$repositoryurlgit" | head -n 1 | grep "HTTP/1.[01] [23].." > /dev/null
		hasgit=$?
	else
		hasgit=2
	fi
	if [ "$hassvn" -eq 0 ]; then
		svn checkout "$repositoryurlsvn" -r HEAD "$rootdir"
		if [ $? -gt 0 ]; then
			echo -e "${RED}Could not checkout IPOPT version ${ipoptversion} from '$repositoryurlsvn'${NC}"
			exit 1
		fi
	elif [ "$hasgit" -eq 0 ]; then
		git clone -b "releases/$ipoptversion" --depth 1 --recurse-submodules "$repositoryurlgit" "$rootdir"
		if [ $? -gt 0 ]; then
			echo -e "${RED}Could not clone IPOPT version ${ipoptversion} from '$repositoryurlgit'${NC}"
			exit 1
		fi
	else
		echo -e "${RED}IPOPT repository for version ${ipoptversion} is not reachable${NC}"
		exit 1
	fi
fi
majorversion="$(echo $ipoptversion | cut -d'.' -f1)"
minorversion="$(echo $ipoptversion | cut -d'.' -f2)"
if [ "$majorversion" -ge 3 ] && [ "$minorversion" -ge 13 ]; then
	# there has been a major change in the build system in version 3.13.0
	echo -e "${RED}build system of IPOPT has changed in 3.13.0 so this is likely to not work as expected${NC}"
	echo -e "${RED}IPOPT does not build successfully because of missing LAPACK and BLAS dependencies${NC}"
	echo -e "${RED}package lapack has to be installed to make compilation work${NC}"
	if [ "$arch" -eq 32 ]; then
		targetarch=--target=i686-w64-mingw32
		MATLABhome=$MATLABhome32
	else
		targetarch=
		MATLABhome=$MATLABhome64
	fi
	configureCOMMON='' #='ADD_CFLAGS="-fopenmp -fexceptions" ADD_CXXFLAGS="-fopenmp -fexceptions" ADD_FFLAGS="-fopenmp -fexceptions -static-libgcc" CDEFS="-DWITHOUT_PTHREAD=1" --disable-shared --with-pic'
	configureCOMMON=${configureCOMMON}$targetarch
	directoryHSL="$rootdir/ThirdParty-HSL"
	directoryBLAS="$rootdir/ThirdParty-Blas"
	directoryLAPACK="$rootdir/ThirdParty-Lapack"
	directoryASL="$rootdir/ThirdParty-ASL"
	directoryMUMPS="$rootdir/ThirdParty-Mumps"
	directoryMETIS="$rootdir/ThirdParty-Metis"
	directoryUtils="$rootdir/CoinUtils"
	directoryMEX="$rootdir/ThirdParty-Mex"
	# directoryMEXOld="$rootdir/build/contrib/MatlabInterface"

	# get and install HSL library
	echo -e "${GREEN}HSL installation${NC}"
	git clone --depth 1 --recurse-submodules "$repositoryurlHSL" "$directoryHSL" 2> /dev/null || (cd "$directoryHSL" || (echo "Could not change directory" && exit 10); git pull)
	if [ $? -gt 0 ]; then
		echo -e "${RED}Could not clone HSL from '$repositoryurlHSL'${NC}"
		exit 1
	fi
	cd "$directoryHSL" || (echo "Could not change directory" && exit 10)
	# check for HSL source code and ask user to copy or accept that it will not be included
	if [ ! -d "$directoryHSL/coinhsl" ]; then
		echo "Download HSL files from 'http://www.hsl.rl.ac.uk/ipopt/'"
		# shellcheck disable=SC2039
		read -p "Copy HSL files to '$directoryHSL/coinhsl' (y) or proceed without HSL (n) " choice
		case "$choice" in
			[jyYJ]* )
				if [ ! -d "$directoryHSL/coinhsl" ]; then
					echo -e "${RED}Files were not copied${NC}"
					exit 1
				fi
				;;
			[nN]* )
				echo -e "${YELLOW}HSL will not be included in IPOPT${NC}"
				;;
			* )
				echo -e "${RED}Terminating${NC}"
				exit 1
				;;
		esac
	fi
	# shellcheck disable=SC2086
	./configure $configureCOMMON
	if [ $? -gt 0 ]; then
		echo -e "${RED}Could not configure HSL${NC}"
		exit 5
	fi
	make -j3
	if [ $? -gt 0 ]; then
		echo -e "${RED}Could not make HSL${NC}"
		exit 5
	fi
	make install
	if [ $? -gt 0 ]; then
		echo -e "${RED}Could not install HSL${NC}"
		exit 5
	fi
	
	# get and install BLAS library
	echo -e "${GREEN}BLAS installation${NC}"
	git clone --depth 1 --recurse-submodules "$repositoryurlBLAS" "$directoryBLAS" 2> /dev/null || (cd "$directoryBLAS" || (echo "Could not change directory" && exit 10); git pull)
	if [ $? -gt 0 ]; then
		echo -e "${RED}Could not clone BLAS from '$repositoryurlBLAS'${NC}"
		exit 1
	fi
	cd "$directoryBLAS" || (echo "Could not change directory" && exit 10)
	sh ./get.Blas
	if [ $? -gt 0 ]; then
		echo -e "${RED}Could not get BLAS${NC}"
		exit 3
	fi
	# shellcheck disable=SC2086
	./configure $configureCOMMON
	if [ $? -gt 0 ]; then
		echo -e "${RED}Could not configure BLAS${NC}"
		exit 5
	fi
	make -j3
	if [ $? -gt 0 ]; then
		echo -e "${RED}Could not make BLAS${NC}"
		exit 5
	fi
	make install
	if [ $? -gt 0 ]; then
		echo -e "${RED}Could not install BLAS${NC}"
		exit 5
	fi
	
	# get and install LAPACK library
	echo -e "${GREEN}LAPACK installation${NC}"
	git clone --depth 1 --recurse-submodules "$repositoryurlLAPACK" "$directoryLAPACK" 2> /dev/null || (cd "$directoryLAPACK" || (echo "Could not change directory" && exit 10); git pull)
	if [ $? -gt 0 ]; then
		echo -e "${RED}Could not clone LAPACK from '$repositoryurlLAPACK'${NC}"
		exit 1
	fi
	cd "$directoryLAPACK" || (echo "Could not change directory" && exit 10)
	sh ./get.Lapack
	if [ $? -gt 0 ]; then
		echo -e "${RED}Could not get LAPACK${NC}"
		exit 3
	fi
	# shellcheck disable=SC2086
	./configure $configureCOMMON
	if [ $? -gt 0 ]; then
		echo -e "${RED}Could not configure LAPACK${NC}"
		exit 5
	fi
	make -j3
	if [ $? -gt 0 ]; then
		echo -e "${RED}Could not make LAPACK${NC}"
		exit 5
	fi
	make install
	if [ $? -gt 0 ]; then
		echo -e "${RED}Could not install LAPACK${NC}"
		exit 5
	fi
	
	# get and install ASL library
	echo -e "${GREEN}ASL installation${NC}"
	git clone --depth 1 --recurse-submodules "$repositoryurlASL" "$directoryASL" 2> /dev/null || (cd "$directoryASL" || (echo "Could not change directory" && exit 10); git pull)
	if [ $? -gt 0 ]; then
		echo -e "${RED}Could not clone ASL from '$repositoryurlASL'"
		exit 1
	fi
	cd "$directoryASL" || (echo "Could not change directory" && exit 10)
	sh ./get.ASL
	if [ $? -gt 0 ]; then
		echo -e "${RED}Could not get ASL${NC}"
		exit 3
	fi
	# shellcheck disable=SC2086
	./configure $configureCOMMON
	if [ $? -gt 0 ]; then
		echo -e "${RED}Could not configure ASL${NC}"
		exit 5
	fi
	make -j3
	if [ $? -gt 0 ]; then
		echo -e "${RED}Could not make ASL${NC}"
		exit 5
	fi
	make install
	if [ $? -gt 0 ]; then
		echo -e "${RED}Could not install ASL${NC}"
		exit 5
	fi
	
	# get and install MUMPS
	echo -e "${GREEN}MUMPS installation${NC}"
	git clone --depth 1 --recurse-submodules "$repositoryurlMUMPS" "$directoryMUMPS" 2> /dev/null || (cd "$directoryMUMPS" || (echo "Could not change directory" && exit 10); git pull)
	if [ $? -gt 0 ]; then
		echo -e "${RED}Could not clone MUMPS from '$repositoryurlMUMPS'${NC}"
		exit 1
	fi
	cd "$directoryMUMPS" || (echo "Could not change directory" && exit 10)
	sh ./get.Mumps
	if [ $? -gt 0 ]; then
		echo -e "${RED}Could not get MUMPS${NC}"
		exit 3
	fi
	# shellcheck disable=SC2086
	./configure $configureCOMMON
	if [ $? -gt 0 ]; then
		echo -e "${RED}Could not configure MUMPS${NC}"
		exit 5
	fi
	make -j3
	if [ $? -gt 0 ]; then
		echo -e "${RED}Could not make MUMPS${NC}"
		exit 5
	fi
	make install
	if [ $? -gt 0 ]; then
		echo -e "${RED}Could not install MUMPS${NC}"
		exit 5
	fi
	
	# get and install METIS
	echo -e "${GREEN}METIS installation${NC}"
	git clone --depth 1 --recurse-submodules "$repositoryurlMETIS" "$directoryMETIS" 2> /dev/null || (cd "$directoryMETIS" || (echo "Could not change directory" && exit 10); git pull)
	if [ $? -gt 0 ]; then
		echo -e "${RED}Could not clone METIS from '$repositoryurlMETIS'${NC}"
		exit 1
	fi
	cd "$directoryMETIS" || (echo "Could not change directory" && exit 10)
	sh ./get.Metis
	if [ $? -gt 0 ]; then
		echo -e "${RED}Could not get METIS${NC}"
		exit 3
	fi
	# shellcheck disable=SC2086
	./configure $configureCOMMON
	if [ $? -gt 0 ]; then
		echo -e "${RED}Could not configure METIS${NC}"
		exit 5
	fi
	make -j3
	if [ $? -gt 0 ]; then
		echo -e "${RED}Could not make METIS${NC}"
		exit 5
	fi
	make install
	if [ $? -gt 0 ]; then
		echo -e "${RED}Could not install METIS${NC}"
		exit 5
	fi
	
	# get and install CoinUtils
	echo -e "${GREEN}CoinUtils installation${NC}"
	# shellcheck disable=SC2063
	latesttag=$(git ls-remote --tags --sort="v:refname" --exit-code --refs "${repositoryurlUtils}" | grep -E -e "*releases*" | sed -E 's/^[[:xdigit:]]+[[:space:]]+refs\/tags\/(.+)/\1/g' | tail -1)
	if [ "$latesttag" -eq "" ]; then
		echo -e "${RED}No release tag found for CoinUtils${NC}"
		exit 1
	fi
	echo -e "${GREEN}Cloning CoinUtils tag '${latesttag}'${NC}"
	git clone --depth 1 --recurse-submodules --branch "$latesttag" "$repositoryurlUtils" "$directoryUtils" 2> /dev/null || (cd "$directoryUtils" || (echo "Could not change directory" && exit 10); git pull && git checkout "$latesttag")
	if [ $? -gt 0 ]; then
		echo "Could not clone CoinUtils from '$repositoryurlUtils'"
		exit 1
	fi
	cd "$directoryUtils" || (echo "Could not change directory" && exit 10)
	# shellcheck disable=SC2086
	./configure -C $configureCOMMON
	if [ $? -gt 0 ]; then
		echo -e "${RED}Could not configure CoinUtils${NC}"
		exit 5
	fi
	make -j3
	if [ $? -gt 0 ]; then
		echo -e "${RED}Could not make CoinUtils${NC}"
		exit 5
	fi
	make install
	if [ $? -gt 0 ]; then
		echo -e "${RED}Could not install CoinUtils${NC}"
		exit 5
	fi
	
	# build IPOPT
	echo -e "${GREEN}IPOPT installation${NC}"
	dependencyfileLAPACK="$directoryLAPACK/coinlapack.pc"
	dependencyfileBLAS="$directoryBLAS/coinblas.pc"
	libdir=""
	libdirLAPACK=""
	liblineLAPACK=""
	libdirBLAS=""
	liblineBLAS=""
	if [ -f "$dependencyfileLAPACK" ]; then
		source /dev/stdin <<EOF
	$(head -3 "$dependencyfileLAPACK")
EOF
		libdirLAPACK="$libdir"
		liblineLAPACK=$(grep '^Libs: ' "$dependencyfileLAPACK" | cut -f2- -d' ')
		liblineLAPACK=$(echo "$liblineLAPACK" | sed -e "s;\${libdir};$libdirLAPACK;")
	else
		echo -e "${RED}LAPACK was not installed correctly${NC}"
		exit 5
	fi
	libdir=""
	if [ -f "$dependencyfileBLAS" ]; then
		source /dev/stdin <<EOF
	$(head -3 "$dependencyfileBLAS")
EOF
		libdirBLAS="$libdir"
		liblineBLAS=$(grep '^Libs: ' "$dependencyfileBLAS" | cut -f2- -d' ')
		liblineBLAS=$(echo "$liblineBLAS" | sed -e "s;\${libdir};$libdirBLAS;")
	else
		echo -e "${RED}BLAS was not installed correctly${NC}"
		exit 5
	fi
	if [ -z "$libdirLAPACK" ] && [ -z "$liblineLAPACK" ]; then
		configureLAPACK=--with-lapack=build
	else
		configureLAPACK='--with-lapack="'
		#configureLAPACK+="-L$libdirLAPACK -L$directoryLAPACK -lf77blas -lcoinblas"
		configureLAPACK+="-L$directoryLAPACK"
		configureLAPACK+='"'
		#configureLAPACK=$(printf '--with-lapack="%s"' $liblineLAPACK)
	fi
	if [ -z "$libdirBLAS" ] && [ -z "$liblineBLAS" ]; then
		configureBLAS=--with-blas=build
	else
		configureBLAS='--with-blas="'
		#configureBLAS+="-L$libdirBLAS -L$directoryBLAS -lf77blas -lcoinblas"
		configureBLAS+="-L$directoryBLAS"
		configureBLAS+='"'
	fi
	configureBLAS=""
	configureLAPACK=""
	cd "$rootdir" || (echo "Could not change directory" && exit 10)
	if [ -d "$rootdir/build" ]; then
		echo -e "${YELLOW}build directory already exists${NC}"
		echo -e "${YELLOW}Maybe it needs to be deleted in order to build successfully${NC}"
	else
		mkdir build
	fi
	if [ ! -d "$rootdir/build/install" ]; then
		mkdir "$rootdir/build/install"
	fi
	cd "$rootdir/build" || (echo "Could not change directory" && exit 10)
	prefix="--prefix=${rootdir}/build/install"
	# shellcheck disable=SC2086
	../configure $prefix --with-matlab-home="$MATLABhome" ADD_CFLAGS="-fopenmp -fexceptions" ADD_CXXFLAGS="-fopenmp -fexceptions" ADD_FFLAGS="-fopenmp -fexceptions -static-libgcc" CDEFS="-DWITHOUT_PTHREAD=1" --enable-static --disable-shared --with-pic --disable-java --disable-sipopt $configureBLAS $configureLAPACK $targetarch
	if [ $? -gt 0 ]; then
		echo -e "${RED}Could not configure IPOPT${NC}"
		exit 5
	fi
	make -j3
	if [ $? -gt 0 ]; then
		echo -e "${RED}Could not make IPOPT${NC}"
		exit 5
	fi
	make test
	if [ $? -gt 0 ]; then
		echo -e "${RED}Could not make test cases${NC}"
		exit 5
	fi
	make install
	if [ $? -gt 0 ]; then
		echo -e "${RED}Could not install IPOPT${NC}"
		exit 5
	fi
	
	# get and install MEX interface
	echo -e "${GREEN}MEX installation${NC}"
	git clone --depth 1 --recurse-submodules "$repositoryurlMEX" "$directoryMEX" 2> /dev/null || (cd "$directoryMEX" || (echo "Could not change directory" && exit 10); git pull)
	if [ $? -gt 0 ]; then
		echo -e "${RED}Could not clone MEX from '$repositoryurlMEX'${NC}"
		exit 1
	fi
	cd "$directoryMEX" || (echo "Could not change directory" && exit 10)
	"$MATLABhome/bin/matlab" -r "addpath(genpath(pwd()));cd('lib');compile_win;exit('force')"
	if [ $? -gt 0 ]; then
		echo "Could not make MEX interface"
		exit 3
	fi
else
	# patch 'Ipopt/configure' if needed
	#if [ -z "${ipoptversion##*$ipoptpatchneeded*}" ]; then
		if [ ! -d "$rootdir/Ipopt" ]; then
			echo -e "${RED}Directory '$rootdir' does not exist${NC}"
			exit 1
		fi
		cd "$rootdir/Ipopt" || (echo "Could not change directory" && exit 10)
		if [ "$hasgit" -eq 0 ]; then
			git apply ../../configure.patch
			if [ $? -gt 0 ]; then
				echo -e "${RED}Could not patch 'configure'${NC}"
			fi
		else
			svn patch ../../configure.patch
			if [ $? -gt 0 ]; then
				echo -e "${RED}Could not patch 'configure'${NC}"
			fi
		fi
	#fi
	cd "$rootdir" || (echo "Could not change directory" && exit 10)
	# check for HSL source code and ask user to copy or accept that it will not be included
	if [ ! -d "$rootdir/ThirdParty/HSL/coinhsl" ]; then
		echo -e "${GREEN}HSL installation${NC}"
		echo "Download HSL files from 'http://www.hsl.rl.ac.uk/ipopt/'"
		# shellcheck disable=SC2039
		read -p "Copy HSL files to 'ThirdParty/HSL/coinhsl' (y) or proceed without HSL (n) " choice
		case "$choice" in
			[jyYJ]* )
				if [ ! -d "$rootdir/ThirdParty/HSL/coinhsl" ]; then
					echo -e "${RED}Files were not copied${NC}"
					exit 1
				fi
				;;
			[nN]* )
				echo -e "${YELLOW}HSL will not be included in IPOPT${NC}"
				;;
			* )
				echo -e "${RED}Terminating${NC}"
				exit 1
				;;
		esac
	fi
	# get libraries
	cd "$rootdir/ThirdParty/Blas/" || (echo "Could not change directory" && exit 10)
	sh get.Blas
	if [ $? -gt 0 ]; then
		echo -e "${RED}Could not get BLAS${NC}"
		exit 3
	fi
	cd "$rootdir/ThirdParty/Lapack" || (echo "Could not change directory" && exit 10)
	sh get.Lapack
	if [ $? -gt 0 ]; then
		echo -e "${RED}Could not get LAPACK${NC}"
		exit 3
	fi
	cd "$rootdir/ThirdParty/Mumps" || (echo "Could not change directory" && exit 10)
	sh get.Mumps
	if [ $? -gt 0 ]; then
		echo -e "${RED}Could not get MUMPS${NC}"
		exit 3
	fi
	cd "$rootdir/ThirdParty/Metis" || (echo "Could not change directory" && exit 10)
	sh get.Metis
	if [ $? -gt 0 ]; then
		echo -e "${RED}Could not get METIS${NC}"
		exit 3
	fi
	cd "$rootdir/Ipopt/contrib/MatlabInterface/" || (echo "Could not change directory" && exit 10)
	sh get.Gnumex
	if [ $? -gt 0 ]; then
		echo -e "${RED}Could not get GNUMEX${NC}"
		exit 3
	fi
	cd "$rootdir/ThirdParty/ASL" || (echo "Could not change directory" && exit 10)
	sh get.ASL
	if [ $? -gt 0 ]; then
		echo -e "${RED}Could not get ASL${NC}"
		exit 3
	fi
	# build IPOPT
	cd "$rootdir" || (echo "Could not change directory" && exit 10)
	if [ -d "$rootdir/build" ]; then
		echo -e "${YELLOW}build directory already exists${NC}"
		echo -e "${YELLOW}Maybe it needs to be deleted in order to build successfully${NC}"
	else
		mkdir build
	fi
	cd "$rootdir/build" || (echo "Could not change directory" && exit 10)
	if [ "$arch" -eq 32 ]; then
		targetarch=--target=i686-w64-mingw32
		MATLABhome=$MATLABhome32
	else
		targetarch=
		MATLABhome=$MATLABhome64
	fi
	# shellcheck disable=SC2086
	../configure --with-matlab-home=$MATLABhome ADD_CFLAGS="-fopenmp -fexceptions" ADD_CXXFLAGS="-fopenmp -fexceptions" ADD_FFLAGS="-fopenmp -fexceptions -static-libgcc" CDEFS="-DWITHOUT_PTHREAD=1" --disable-shared --with-pic --with-blas=BUILD --with-lapack=BUILD $targetarch
	if [ $? -gt 0 ]; then
		echo -e "${RED}Could not configure IPOPT${NC}"
		exit 5
	fi
	make -j3
	if [ $? -gt 0 ]; then
		echo -e "${RED}Could not make IPOPT${NC}"
		exit 5
	fi
	make test
	if [ $? -gt 0 ]; then
		echo -e "${RED}Could not make test cases${NC}"
		exit 5
	fi
	make install
	if [ $? -gt 0 ]; then
		echo -e "${RED}Could not install IPOPT${NC}"
		exit 5
	fi
	# build mex file
	cd "$rootdir/build/Ipopt/contrib/MatlabInterface/src/" || (echo "Could not change directory" && exit 10)
	make gnumex
	if [ $? -gt 0 ]; then
		echo -e "${RED}Could not configure MATLAB${NC}"
		exit 5
	fi
	make mexopts
	if [ $? -gt 0 ]; then
		echo -e "${RED}Could not make mexopts${NC}"
		exit 5
	fi
	sed -i 's/\(set GM_ADD_LIBS=\)\(-llibmx -llibmex -llibmat.*\)/\1-static \2/' 'mexopts.bat'
	if [ $? -gt 0 ]; then
		echo -e "${RED}Could not edit mexopts${NC}"
		exit 5
	fi
	make install
	if [ $? -gt 0 ]; then
		echo -e "${RED}Could not make mex file${NC}"
		exit 5
	fi
fi
