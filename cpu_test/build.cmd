@echo off

cl -I..\code -DDEBUG -Z7 ..\code\sabre_svo.cc sabre_sw_renderer.cc -Fesw.exe
