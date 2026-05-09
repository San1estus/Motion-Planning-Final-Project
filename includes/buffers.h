#pragma once
#include "config.h"

// Global variables for VAOs and VBOs
extern GLuint TreeVAO, TreeVBO;
extern GLuint PathVAO, PathVBO;
extern GLuint pointsVAO, pointsVBO;
extern GLuint carVAO, carVBO, carEBO;
extern unsigned int rectangleIndices[6];
// Buffers management functions
void setupBuffers() {
    glGenVertexArrays(1, &TreeVAO);
    glGenBuffers(1, &TreeVBO);
    glBindVertexArray(TreeVAO);
    glBindBuffer(GL_ARRAY_BUFFER, TreeVBO);
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 2 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);

    glGenVertexArrays(1, &PathVAO);
    glGenBuffers(1, &PathVBO);
    glBindVertexArray(PathVAO);
    glBindBuffer(GL_ARRAY_BUFFER, PathVBO);
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 2 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);

    glGenVertexArrays(1, &pointsVAO);
    glGenBuffers(1, &pointsVBO);
    glBindVertexArray(pointsVAO);
    glBindBuffer(GL_ARRAY_BUFFER, pointsVBO);
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 2 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);

    glGenVertexArrays(1, &carVAO);
    glGenBuffers(1, &carVBO);
    glGenBuffers(1, &carEBO);   
    glBindVertexArray(carVAO);
    glBindBuffer(GL_ARRAY_BUFFER, carVBO);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, carEBO);

    glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(rectangleIndices), rectangleIndices, GL_STATIC_DRAW);
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 2 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);
}

void updateTreeBuffers(const std::vector<float>& vertices) {
    glBindBuffer(GL_ARRAY_BUFFER, TreeVBO);
    glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(float), vertices.data(), GL_DYNAMIC_DRAW);
}

void updatePathBuffers(const std::vector<float>& vertices) {
    glBindBuffer(GL_ARRAY_BUFFER, PathVBO);
    glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(float), vertices.data(), GL_DYNAMIC_DRAW);
}