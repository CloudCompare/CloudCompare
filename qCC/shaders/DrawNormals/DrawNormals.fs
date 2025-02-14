#version 330 core

uniform mediump vec4 color;

out vec4 OutputColor;

void main(void)
{
    OutputColor = color;
}
