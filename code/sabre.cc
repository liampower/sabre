#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

#include <glad/glad.h>
#include <glfw/glfw3.h>

#include "sabre.h"


extern int
main(int ArgCount, const char** const Args)
{
    if (GLFW_FALSE == glfwInit())
    {
        fprintf(stderr, "Failed to initialise GLFW\n");
    }

    fprintf(stdout, "Output\n");

    
    glfwTerminate();

    return EXIT_SUCCESS;
}


