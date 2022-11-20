#!/bin/bash

echo -e "\n*** OpenGL Mesa Environment Vars"
echo "printenv | grep MESA"
printenv | grep MESA
echo -e "\n*** glxinfo | grep -i opengl"
glxinfo | grep -i opengl

