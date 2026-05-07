#pragma once
#include "config.h"
#include <string>
#include <fstream>
#include <sstream>

std::string loadShaderSource(const std::string& filepath) {
    std::ifstream file(filepath);
    std::stringstream ss;
    if (!file.is_open()) {
        std::cerr << "Failed to open shader file: " << filepath << std::endl;
        return "";
    }
    ss << file.rdbuf();
    file.close();
    std::string source = ss.str();
    return source;
}

inline GLuint createShaderProgram(const char* vertSrc, const char* fragSrc) {
    GLuint vert = glCreateShader(GL_VERTEX_SHADER);
    glShaderSource(vert, 1, &vertSrc, NULL);
    glCompileShader(vert);

    GLuint frag = glCreateShader(GL_FRAGMENT_SHADER);
    glShaderSource(frag, 1, &fragSrc, NULL);
    glCompileShader(frag);

    GLuint prog = glCreateProgram();
    glAttachShader(prog, vert);
    glAttachShader(prog, frag);
    glLinkProgram(prog);

    glDeleteShader(vert);
    glDeleteShader(frag);
    return prog;
}