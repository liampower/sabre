@echo off
title VC Shell
doskey make=nmake -nologo $*
cmd /k call "C:\Program Files (x86)\Microsoft Visual Studio\2017\Community\VC\Auxiliary\Build\vcvarsall.bat" x64

