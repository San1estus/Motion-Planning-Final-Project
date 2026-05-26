#version 330 core
layout (location = 0) in vec2 aPos;

void main() {
    float aspect =1920.0 / 1080.0;
    gl_Position = vec4(aPos.x / aspect, aPos.y, 0.0, 1.0);
}