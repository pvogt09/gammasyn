#!/bin/sh
# download, configure and build IPOPT mex file with MinGW (in a Msys2 console)
# configuration
ipoptversion='3.12.13'
arch=64
basedir="/D/Projekt/Implementierung/lib/optimization/IPOPT"
repositoryurlsvn="https://projects.coin-or.org/svn/Ipopt/releases/$ipoptversion/"
repositoryurlgit="https://github.com/coin-or/Ipopt"
repositoryurlBLAS="https://github.com/coin-or-tools/ThirdParty-Blas"
repositoryurlLAPACK="https://github.com/coin-or-tools/ThirdParty-Lapack"
repositoryurlASL="https://github.com/coin-or-tools/ThirdParty-ASL.git"
repositoryurlHSL="https://github.com/coin-or-tools/ThirdParty-HSL.git"
repositoryurlMUMPS="https://github.com/coin-or-tools/ThirdParty-Mumps.git"
repositoryurlMETIS="https://github.com/coin-or-tools/ThirdParty-Metis.git"
repositoryurlMEX="https://github.com/ebertolazzi/mexIPOPT"
MATLABhome64=/c/Progra~1/MATLAB/R2015b
MATLABhome32=/c/Progra~2/MATLAB/R2015b

## beginning of script
rootdir="$basedir/CoinIpopt_$ipoptversion"
if [ "$arch" -eq 32 ]; then
	rootdir="${rootdir}_32bit"
fi
rootdir="${rootdir}/"
# version that needs the patch 'configure.patch'
ipoptpatchneeded='3.12.10'
cd "$basedir"
# get requested IPOPT version and checkout/clone into working directory
hasgit=0
hassvn=0
if [ ! -d "$rootdir" ]; then
	mkdir "$rootdir"
	if [ $? -gt 0 ]; then
		echo "Could not create directory for IPOPT"
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
			echo "Could not checkout IPOPT version ${ipoptversion} from '$repositoryurlsvn'"
			exit 1
		fi
	elif [ "$hasgit" -eq 0 ]; then
		git clone -b "releases/$ipoptversion" --depth 1 --recurse-submodules "$repositoryurlgit" "$rootdir"
		if [ $? -gt 0 ]; then
			echo "Could not clone IPOPT version ${ipoptversion} from '$repositoryurlgit'"
			exit 1
		fi
	else
		echo "IPOPT repository for version ${ipoptversion} is not reachable"
		exit 1
	fi
fi
majorversion="$(cut -d'.' -f1 <<<$ipoptversion)"
minorversion="$(cut -d'.' -f2 <<<$ipoptversion)"
if [ "$majorversion" -ge 3 -a "$minorversion" -ge 13 ]; then
	# there has been a major change in the build system in version 3.13.0
	echo 'build system of IPOPT has changed in 3.13.0 so this is likely to not work as expected'
	echo 'IPOPT does not build successfully because of missing LAPACK and BLAS dependencies'
	echo 'package lapack has to be installed to make compilation work'
	if [ "$arch" -eq 32 ]; then
		targetarch=--target=i686-w64-mingw32
		MATLABhome=$MATLABhome32
	else
		targetarch=
		MATLABhome=$MATLABhome64
	fi
	configureCOMMON='' #='ADD_CFLAGS="-fopenmp -fexceptions" ADD_CXXFLAGS="-fopenmp -fexceptions" ADD_FFLAGS="-fopenmp -fexceptions -static-libgcc" CDEFS="-DWITHOUT_PTHREAD=1" --disable-shared --with-pic'
	configureCOMMON+=$targetarch
	directoryHSL="$rootdir/ThirdParty-HSL"
	directoryBLAS="$rootdir/ThirdParty-Blas"
	directoryLAPACK="$rootdir/ThirdParty-Lapack"
	directoryASL="$rootdir/ThirdParty-ASL"
	directoryMUMPS="$rootdir/ThirdParty-Mumps"
	directoryMETIS="$rootdir/ThirdParty-Metis"
	directoryMEX="$rootdir/ThirdParty-Mex"
	# get and install HSL library
	echo "HSL installation"
	git clone --depth 1 --recurse-submodules "$repositoryurlHSL" "$directoryHSL" 2> /dev/null || (cd "$directoryHSL"; git pull)
	if [ $? -gt 0 ]; then
		echo "Could not clone HSL from '$repositoryurlHSL'"
		exit 1
	fi
	cd "$directoryHSL"
	# check for HSL source code and ask user to copy or accept that it will not be included
	if [ ! -d "$directoryHSL/coinhsl" ]; then
		echo "Download HSL files from 'http://www.hsl.rl.ac.uk/ipopt/'"
		read -p "Copy HSL files to '$directoryHSL/coinhsl' (y) or proceed without HSL (n) " choice
		case "$choice" in
			[jyYJ]* )
				if [ ! -d "$directoryHSL/coinhsl" ]; then
					echo "Files were not copied"
					exit 1
				fi
				;;
			[nN]* )
				echo "HSL will not be included in IPOPT"
				;;
			* )
				echo "Terminating"
				exit 1
				;;
		esac
	fi
	./configure $configureCOMMON
	if [ $? -gt 0 ]; then
		echo "Could not configure HSL"
		exit -2
	fi
	make -j3
	if [ $? -gt 0 ]; then
		echo "Could not make HSL"
		exit -2
	fi
	make install
	if [ $? -gt 0 ]; then
		echo "Could not install HSL"
		exit -2
	fi
	
	# get and install BLAS library
	echo "BLAS installation"
	git clone --depth 1 --recurse-submodules "$repositoryurlBLAS" "$directoryBLAS" 2> /dev/null || (cd "$directoryBLAS"; git pull)
	if [ $? -gt 0 ]; then
		echo "Could not clone BLAS from '$repositoryurlBLAS'"
		exit 1
	fi
	cd "$directoryBLAS"
	sh ./get.Blas
	if [ $? -gt 0 ]; then
		echo "Could not get BLAS"
		exit -1
	fi
	./configure $configureCOMMON
	if [ $? -gt 0 ]; then
		echo "Could not configure BLAS"
		exit -2
	fi
	make -j3
	if [ $? -gt 0 ]; then
		echo "Could not make BLAS"
		exit -2
	fi
	make install
	if [ $? -gt 0 ]; then
		echo "Could not install BLAS"
		exit -2
	fi
	
	# get and install LAPACK library
	echo "LAPACK installation"
	git clone --depth 1 --recurse-submodules "$repositoryurlLAPACK" "$directoryLAPACK" 2> /dev/null || (cd "$directoryLAPACK"; git pull)
	if [ $? -gt 0 ]; then
		echo "Could not clone LAPACK from '$repositoryurlLAPACK'"
		exit 1
	fi
	cd "$directoryLAPACK"
	sh ./get.Lapack
	if [ $? -gt 0 ]; then
		echo "Could not get LAPACK"
		exit -1
	fi
	./configure $configureCOMMON
	if [ $? -gt 0 ]; then
		echo "Could not configure LAPACK"
		exit -2
	fi
	make -j3
	if [ $? -gt 0 ]; then
		echo "Could not make LAPACK"
		exit -2
	fi
	make install
	if [ $? -gt 0 ]; then
		echo "Could not install LAPACK"
		exit -2
	fi
	
	# get and install ASL library
	echo "ASL installation"
	git clone --depth 1 --recurse-submodules "$repositoryurlASL" "$directoryASL" 2> /dev/null || (cd "$directoryASL"; git pull)
	if [ $? -gt 0 ]; then
		echo "Could not clone ASL from '$repositoryurlASL'"
		exit 1
	fi
	cd "$directoryASL"
	sh ./get.ASL
	if [ $? -gt 0 ]; then
		echo "Could not get ASL"
		exit -1
	fi
	./configure $configureCOMMON
	if [ $? -gt 0 ]; then
		echo "Could not configure ASL"
		exit -2
	fi
	make -j3
	if [ $? -gt 0 ]; then
		echo "Could not make ASL"
		exit -2
	fi
	make install
	if [ $? -gt 0 ]; then
		echo "Could not install ASL"
		exit -2
	fi
	
	# get and install MUMPS
	echo "MUMPS installation"
	git clone --depth 1 --recurse-submodules "$repositoryurlMUMPS" "$directoryMUMPS" 2> /dev/null || (cd "$directoryMUMPS"; git pull)
	if [ $? -gt 0 ]; then
		echo "Could not clone MUMPS from '$repositoryurlMUMPS'"
		exit 1
	fi
	cd "$directoryMUMPS"
	sh ./get.Mumps
	if [ $? -gt 0 ]; then
		echo "Could not get MUMPS"
		exit -1
	fi
	./configure $configureCOMMON
	if [ $? -gt 0 ]; then
		echo "Could not configure MUMPS"
		exit -2
	fi
	make -j3
	if [ $? -gt 0 ]; then
		echo "Could not make MUMPS"
		exit -2
	fi
	make install
	if [ $? -gt 0 ]; then
		echo "Could not install MUMPS"
		exit -2
	fi
	
	# get and install METIS
	echo "METIS installation"
	git clone --depth 1 --recurse-submodules "$repositoryurlMETIS" "$directoryMETIS" 2> /dev/null || (cd "$directoryMETIS"; git pull)
	if [ $? -gt 0 ]; then
		echo "Could not clone METIS from '$repositoryurlMETIS'"
		exit 1
	fi
	cd "$directoryMETIS"
	sh ./get.Metis
	if [ $? -gt 0 ]; then
		echo "Could not get METIS"
		exit -1
	fi
	./configure $configureCOMMON
	if [ $? -gt 0 ]; then
		echo "Could not configure METIS"
		exit -2
	fi
	make -j3
	if [ $? -gt 0 ]; then
		echo "Could not make METIS"
		exit -2
	fi
	make install
	if [ $? -gt 0 ]; then
		echo "Could not install METIS"
		exit -2
	fi
	# build IPOPT
	echo "IPOPT installation"
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
		liblineLAPACK=$(echo $liblineLAPACK | sed -e "s;\${libdir};$libdirLAPACK;")
	else
		echo "LAPACK was not installed correctly"
		exit -2
	fi
	libdir=""
	if [ -f "$dependencyfileBLAS" ]; then
		source /dev/stdin <<EOF
	$(head -3 "$dependencyfileBLAS")
EOF
		libdirBLAS="$libdir"
		liblineBLAS=$(grep '^Libs: ' "$dependencyfileBLAS" | cut -f2- -d' ')
		liblineBLAS=$(echo $liblineBLAS | sed -e "s;\${libdir};$libdirBLAS;")
	else
		echo "BLAS was not installed correctly"
		exit -2
	fi
	if [ -z "$libdirLAPACK" -a -z "$liblineLAPACK" ]; then
		configureLAPACK=--with-lapack=build
	else
		configureLAPACK='--with-lapack="'
		#configureLAPACK+="-L$libdirLAPACK -L$directoryLAPACK -lf77blas -lcoinblas"
		configureLAPACK+="-L$directoryLAPACK"
		configureLAPACK+='"'
		#configureLAPACK=$(printf '--with-lapack="%s"' $liblineLAPACK)
	fi
	if [ -z "$libdirBLAS" -a -z "$liblineBLAS" ]; then
		configureBLAS=--with-blas=build
	else
		configureBLAS='--with-blas="'
		#configureBLAS+="-L$libdirBLAS -L$directoryBLAS -lf77blas -lcoinblas"
		configureBLAS+="-L$directoryBLAS"
		configureBLAS+='"'
	fi
	configureBLAS=""
	configureLAPACK=""
	cd "$rootdir"
	if [ -d "$rootdir/build" ]; then
		echo "build directory already exists"
		echo "Maybe it needs to be deleted in order to build successfully"
	else
		mkdir build
	fi
	cd "$rootdir/build"
	../configure --with-matlab-home=$MATLABhome ADD_CFLAGS="-fopenmp -fexceptions" ADD_CXXFLAGS="-fopenmp -fexceptions" ADD_FFLAGS="-fopenmp -fexceptions -static-libgcc" CDEFS="-DWITHOUT_PTHREAD=1" --disable-shared --with-pic $configureBLAS $configureLAPACK $targetarch
	if [ $? -gt 0 ]; then
		echo "Could not configure IPOPT"
		exit -2
	fi
	make -j3
	if [ $? -gt 0 ]; then
		echo "Could not make IPOPT"
		exit -2
	fi
	make test
	if [ $? -gt 0 ]; then
		echo "Could not make test cases"
		exit -2
	fi
	make install
	if [ $? -gt 0 ]; then
		echo "Could not install IPOPT"
		exit -2
	fi
	
	# get and install MEX interface
	echo "MEX installation"
	git clone --depth 1 --recurse-submodules "$repositoryurlMEX" "$directoryMEX" 2> /dev/null || (cd "$directoryMEX"; git pull)
	if [ $? -gt 0 ]; then
		echo "Could not clone MEX from '$repositoryurlMEX'"
		exit 1
	fi
	cd "$directoryMEX"
	$MATLABhome/bin/matlab -r "addpath(genpath(pwd()));cd('lib');compile_win;exit('force')"
	if [ $? -gt 0 ]; then
		echo "Could not make MEX interface"
		exit -1
	fi
	exit 0
else
	# patch 'Ipopt/configure' if needed
	#if [ -z "${ipoptversion##*$ipoptpatchneeded*}" ]; then
		if [ ! -d "$rootdir/Ipopt" ]; then
			echo "Directory '$rootdir' does not exist"
			exit 1
		fi
		cd "$rootdir/Ipopt"
		if [ "$hasgit" -eq 0 ]; then
			git apply ../../configure.patch
			if [ $? -gt 0 ]; then
				echo "Could not patch 'configure'"
			fi
		else
			svn patch ../../configure.patch
			if [ $? -gt 0 ]; then
				echo "Could not patch 'configure'"
			fi
		fi
	#fi
	cd "$rootdir"
	# check for HSL source code and ask user to copy or accept that it will not be included
	if [ ! -d "$rootdir/ThirdParty/HSL/coinhsl" ]; then
		echo "HSL installation"
		echo "Download HSL files from 'http://www.hsl.rl.ac.uk/ipopt/'"
		read -p "Copy HSL files to 'ThirdParty/HSL/coinhsl' (y) or proceed without HSL (n) " choice
		case "$choice" in
			[jyYJ]* )
				if [ ! -d "$rootdir/ThirdParty/HSL/coinhsl" ]; then
					echo "Files were not copied"
					exit 1
				fi
				;;
			[nN]* )
				echo "HSL will not be included in IPOPT"
				;;
			* )
				echo "Terminating"
				exit 1
				;;
		esac
	fi
	# get libraries
	cd "$rootdir/ThirdParty/Blas/"
	sh get.Blas
	if [ $? -gt 0 ]; then
		echo "Could not get BLAS"
		exit -1
	fi
	cd "$rootdir/ThirdParty/Lapack"
	sh get.Lapack
	if [ $? -gt 0 ]; then
		echo "Could not get LAPACK"
		exit -1
	fi
	cd "$rootdir/ThirdParty/Mumps"
	sh get.Mumps
	if [ $? -gt 0 ]; then
		echo "Could not get MUMPS"
		exit -1
	fi
	cd "$rootdir/ThirdParty/Metis"
	sh get.Metis
	if [ $? -gt 0 ]; then
		echo "Could not get METIS"
		exit -1
	fi
	cd "$rootdir/Ipopt/contrib/MatlabInterface/"
	sh get.Gnumex
	if [ $? -gt 0 ]; then
		echo "Could not get GNUMEX"
		exit -1
	fi
	cd "$rootdir/ThirdParty/ASL"
	sh get.ASL
	if [ $? -gt 0 ]; then
		echo "Could not get ASL"
		exit -1
	fi
	# build IPOPT
	cd "$rootdir"
	if [ -d "$rootdir/build" ]; then
		echo "build directory already exists"
		echo "Maybe it needs to be deleted in order to build successfully"
	else
		mkdir build
	fi
	cd "$rootdir/build"
	if [ "$arch" -eq 32 ]; then
		targetarch=--target=i686-w64-mingw32
		MATLABhome=$MATLABhome32
	else
		targetarch=
		MATLABhome=$MATLABhome64
	fi
	../configure --with-matlab-home=$MATLABhome ADD_CFLAGS="-fopenmp -fexceptions" ADD_CXXFLAGS="-fopenmp -fexceptions" ADD_FFLAGS="-fopenmp -fexceptions -static-libgcc" CDEFS="-DWITHOUT_PTHREAD=1" --disable-shared --with-pic --with-blas=BUILD --with-lapack=BUILD $targetarch
	if [ $? -gt 0 ]; then
		echo "Could not configure IPOPT"
		exit -2
	fi
	make -j3
	if [ $? -gt 0 ]; then
		echo "Could not make IPOPT"
		exit -2
	fi
	make test
	if [ $? -gt 0 ]; then
		echo "Could not make test cases"
		exit -2
	fi
	make install
	if [ $? -gt 0 ]; then
		echo "Could not install IPOPT"
		exit -2
	fi
	# build mex file
	cd "$rootdir/build/Ipopt/contrib/MatlabInterface/src/"
	make gnumex
	if [ $? -gt 0 ]; then
		echo "Could not configure MATLAB"
		exit -2
	fi
	make mexopts
	if [ $? -gt 0 ]; then
		echo "Could not make mexopts"
		exit -2
	fi
	sed -i 's/\(set GM_ADD_LIBS=\)\(-llibmx -llibmex -llibmat.*\)/\1-static \2/' 'mexopts.bat'
	if [ $? -gt 0 ]; then
		echo "Could not edit mexopts"
		exit -2
	fi
	make install
	if [ $? -gt 0 ]; then
		echo "Could not make mex file"
		exit -2
	fi
fi
