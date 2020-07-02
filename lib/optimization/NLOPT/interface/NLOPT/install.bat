@echo off
if "%~1"=="32" (
	"C:\Program Files (x86)\Microsoft Visual Studio 14.0\VC\bin\lib" /def:libnlopt-0-32.def /MACHINE:X86
	mex nlopt_optimize.c libnlopt-0-32.lib -I.
) else (
	"C:\Program Files (x86)\Microsoft Visual Studio 14.0\VC\bin\lib" /def:libnlopt-0-64.def /MACHINE:X64
	mex nlopt_optimize.c libnlopt-0-64.lib -I.
)