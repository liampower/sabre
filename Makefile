Sources=code/sabre.cc
Objects=sabre.o
Executable=sabre.exe
ShaderPacker=shader_packer.exe

Libraries=glad.lib glfw3.lib opengl32.lib kernel32.lib user32.lib gdi32.lib shell32.lib
Compiler=clang
Linker=link -nologo
DisabledWarnings=-Wno-unused-parameter -Wno-c++98-compat -Wno-gnu-anonymous-struct -Wno-old-style-cast
CompilerFlags=-isystem include -Wall -Wextra -Weverything -Wpadded -std=c++14 -mavx -fno-omit-frame-pointer -fno-exceptions -fno-rtti -ffast-math -g -gcodeview $(DisabledWarnings)
LinkerFlags=-incremental:no -debug -entry:mainCRTStartup -libpath:libs -machine:x64 -nodefaultlib:libcmt $(Libraries)

all: $(Executable)

$(Objects): $(Sources)
	@$(Compiler) -c $(CompilerFlags) $(Sources)

$(Executable): $(Objects)  tags
	@$(Linker) $(Objects) $(LinkerFlags) -out:$(Executable)

clean:
	@del $(Objects) 
	@del tags
	@del $(Executable)
	@del *.pdb

tags:
	@ctags -R code

run: $(Executable)
	@$(Executable)
