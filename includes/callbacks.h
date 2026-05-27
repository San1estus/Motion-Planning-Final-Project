#pragma once
#include "config.h"
#include "rrt.h"

void mouseButtonCallback(GLFWwindow* window, int button, int action, int mods) {
    AppState* state = (AppState*)glfwGetWindowUserPointer(window);
    
    double xpos, ypos;
    glfwGetCursorPos(window, &xpos, &ypos);
    int width, height;
    glfwGetWindowSize(window, &width, &height);
    float xworld = ((float)xpos / width * 2.0f - 1.0f) * aspect;
    float yworld = 1.0f - (float)ypos / height * 2.0f;
    
    if (state->obstacleMode) {
        if (button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_PRESS) {
            state->dragging = true;
            state->obsInitX = xworld;
            state->obsInitY = yworld;
            state->dragX = xworld;
            state->dragY = yworld;
        } else if (button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_RELEASE) {
            Obstacle obs;
            obs.minX = std::min(state->obsInitX, xworld);
            obs.maxX = std::max(state->obsInitX, xworld);
            obs.minY = std::min(state->obsInitY, yworld);
            obs.maxY = std::max(state->obsInitY, yworld);
            obs.minX = std::max(obs.minX, -aspect + padding);
            obs.maxX = std::min(obs.maxX,  aspect - padding);
            obs.minY = std::max(obs.minY, -1.0f + padding);
            obs.maxY = std::min(obs.maxY,  1.0f - padding);
            state->obstacles.push_back(obs);
            state->dragging = false;
        }
    } else {
        if (button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_PRESS) {
            state->initAndGoalVertices[0] = xworld;
            state->initAndGoalVertices[1] = yworld;
            state->startSet = true;
            std::cout << "Start set at: (" << xworld << ", " << yworld << ")" << std::endl;
        }
        if (button == GLFW_MOUSE_BUTTON_RIGHT && action == GLFW_PRESS) {
            state->initAndGoalVertices[2] = xworld;
            state->initAndGoalVertices[3] = yworld;
            state->goalSet = true;
            std::cout << "Goal set at: (" << xworld << ", " << yworld << ")" << std::endl;
        }
    }
}

void keyCallback(GLFWwindow* window, int key, int scancode, int action, int mods) {
    AppState* state = (AppState*)glfwGetWindowUserPointer(window);
    
    switch (key) {
        case GLFW_KEY_R:
            if (action == GLFW_PRESS) {
                state->rrt.nodes.clear();
                state->rrt.kdTree.clear();
                state->rrt.currIter = 0;
                state->goalNode = nullptr;
                state->path.clear();
                state->pathVertices.clear();
                state->pathFound = false;
                state->pathIndex = 0;
                state->initAndGoalVertices = {0.0f, 0.0f, 0.0f, 0.0f};
                state->startAlgorithm = false;
                state->goalSet = false;
                state->dragging = false;
                state->obstacles.clear();
            }
            break;
        case GLFW_KEY_SPACE:
            if (action == GLFW_PRESS) {
                if(state->startSet && state->goalSet) state->startAlgorithm = true;
                std::cout <<"Space pressed: " << state->startAlgorithm << std::endl;
                std::cout << "obstacleMode: " << state->obstacleMode << std::endl;
                state->rrt.init(-aspect, aspect, -1.0f, 1.0f, state->initAndGoalVertices[0], state->initAndGoalVertices[1], state->initAndGoalVertices[2], state->initAndGoalVertices[3], state->obstacles);
            }
            break;
        case GLFW_KEY_ESCAPE:
            if (action == GLFW_PRESS)
                glfwSetWindowShouldClose(window, true);
            break;
        case GLFW_KEY_O:
            if (action == GLFW_PRESS) {
                state->obstacleMode = !state->obstacleMode;
            }
            break;
    }
}

void cursorPosCallback(GLFWwindow* window, double xpos, double ypos) {
    AppState* state = (AppState*)glfwGetWindowUserPointer(window);
    
    if (state->dragging) {
        int width, height;
        glfwGetWindowSize(window, &width, &height);
        float xworld = ((float)xpos / width * 2.0f - 1.0f) * aspect;
        float yworld = 1.0f - (float)ypos / height * 2.0f;
        state->dragX = xworld;
        state->dragY = yworld;
    }
}