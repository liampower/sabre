# Created by and for Qt Creator This file was created for editing the project sources only.
# You may attempt to use it for building too, by modifying this file here.

#TARGET = sabre

QT = core gui widgets

HEADERS = \
   $$PWD/code/noise_gen.hh \
   $$PWD/code/render.hh \
   $$PWD/code/sabre.hh \
   $$PWD/code/svo.hh \
   $$PWD/code/vecmath.hh \
   $$PWD/include/glad/glad.h \
   $$PWD/include/glfw/glfw3.h \
   $$PWD/include/glfw/glfw3native.h \
   $$PWD/include/imgui/imconfig.h \
   $$PWD/include/imgui/imgui.h \
   $$PWD/include/imgui/imgui_impl_glfw.h \
   $$PWD/include/imgui/imgui_impl_opengl3.h \
   $$PWD/include/imgui/imgui_internal.h \
   $$PWD/include/imgui/imstb_rectpack.h \
   $$PWD/include/imgui/imstb_textedit.h \
   $$PWD/include/imgui/imstb_truetype.h \
   $$PWD/include/KHR/khrplatform.h \
   $$PWD/include/cgltf.h \
   $$PWD/include/nanoprofile.h \
   $$PWD/include/stb_image.h

SOURCES = \
   $$PWD/code/glad.c \
   $$PWD/code/htable_cs.glsl \
   $$PWD/code/import.cc \
   $$PWD/code/main_fs.glsl \
   $$PWD/code/main_vs.glsl \
   $$PWD/code/noise_gen.cc \
   $$PWD/code/render.cc \
   $$PWD/code/render_cs.glsl \
   $$PWD/code/sabre_linux.cc \
   $$PWD/code/sabre_windows.cc \
   $$PWD/code/svo.cc \
   $$PWD/misc/maxdn_finder.cc \
   $$PWD/misc/shader_compiler.cc \
   $$PWD/build.ninja \
   $$PWD/build_windows.ninja \
   $$PWD/LICENSE \
   $$PWD/README.MD

INCLUDEPATH = \
    $$PWD/include \
    $$PWD/include/glad \
    $$PWD/include/glfw \
    $$PWD/include/imgui \
    $$PWD/include/KHR

#DEFINES = 

