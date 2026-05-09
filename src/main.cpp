/*RRT Implementation for a simple car*/

#include "config.h"
#include "shader_utils.h"
#include "rrt.h"
#include "buffers.h"
#include "callbacks.h"

// Car dimensions
float carLength = 0.1f;
float carWidth = 2/3.0f * carLength;

std::vector<float> getCarVertices(float cx, float cy, float theta, float w, float h) {
    std::vector<std::pair<float, float>> localVertices = {
        {w/2, h/2},
        {w/2, -h/2},
        {-w/2, -h/2},
        {-w/2, h/2}
    };
    for (auto& vertex : localVertices) {
        float x = vertex.first;
        float y = vertex.second;
        vertex.first = x * cos(theta) - y * sin(theta);
        vertex.second = x * sin(theta) + y * cos(theta);
    }
    for (auto& vertex : localVertices) {
        vertex.first += cx;
        vertex.second += cy;
    }
    std::vector<float> vertices;
    for (const auto& vertex : localVertices) {
        vertices.push_back(vertex.first);
        vertices.push_back(vertex.second);
    }
    return vertices;
}

unsigned int rectangleIndices[] = {
    0, 1, 2,
    2, 3, 0
};

// Global variables for VAOs and VBOs
GLuint TreeVAO, TreeVBO;
GLuint PathVAO, PathVBO;
GLuint pointsVAO, pointsVBO;
GLuint carVAO, carVBO, carEBO;


int main() {
    GLFWwindow* window;

    if(!glfwInit()) {
        std::cerr << "Failed to initialize GLFW" << std::endl;
        return -1;
    }

    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    window = glfwCreateWindow(800, 600, "RRT", NULL, NULL);
    glfwMakeContextCurrent(window);
    
    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
        glfwTerminate();
        std::cerr << "Failed to initialize GLAD" << std::endl;
        return -1;  
    }
    
    std::string vertSrc = loadShaderSource(std::string(SHADER_DIR) + "vertex.glsl");
    std::string fragSrc = loadShaderSource(std::string(SHADER_DIR) + "fragment.glsl");
    GLuint shaderProgram = createShaderProgram(vertSrc.c_str(), fragSrc.c_str());
    GLint colorLoc = glGetUniformLocation(shaderProgram, "uColor");
    
    std::vector<float> carVertices;
    setupBuffers(); 
    glPointSize(10.0f);
    float deltaTime = 0.0f;
    float lastFrame = 0.0f;

    AppState state;
    glfwSetWindowUserPointer(window, &state);
    glfwSetMouseButtonCallback(window, mouseButtonCallback);
    glfwSetKeyCallback(window, keyCallback);
    glfwSetCursorPosCallback(window, cursorPosCallback);
    
    glBindBuffer(GL_ARRAY_BUFFER, pointsVBO); 
    glBufferData(GL_ARRAY_BUFFER, state.initAndGoalVertices.size() * sizeof(float), state.initAndGoalVertices.data(), GL_STATIC_DRAW);
    while(!glfwWindowShouldClose(window)) {
        
        float currentFrame = glfwGetTime();
        deltaTime = currentFrame - lastFrame;
        glClear(GL_COLOR_BUFFER_BIT);
        glUseProgram(shaderProgram);
        
        std::vector<float> edgeVertices;
        if(state.startAlgorithm && !state.obstacleMode){
            if(!state.goalNode){
                state.goalNode = state.rrt.buildStar(state.initAndGoalVertices[2], state.initAndGoalVertices[3], 0.05f, 10000, 0.01f, 0.2f, state.obstacles);
                if(state.goalNode){
                    state.path = state.rrt.getPath(state.goalNode);
                }
                edgeVertices = state.rrt.getEdgeVertices();
                updateTreeBuffers(edgeVertices);
            } else{
                edgeVertices = state.rrt.getEdgeVertices();
            }
        
            glUniform4f(colorLoc, 1.0f, 0.0f, 0.0f, 1.0f);
            glBindVertexArray(TreeVAO);
            glDrawArrays(GL_LINES, 0, edgeVertices.size() / 2);

            if(!state.path.empty()){
                glUniform4f(colorLoc, 0.0f, 1.0f, 0.0f, 1.0f);
                if(!state.pathFound){
                    for(size_t i = 1; i < state.path.size(); ++i){
                        state.pathVertices.push_back(state.path[i-1]->x);
                        state.pathVertices.push_back(state.path[i-1]->y);
                        state.pathVertices.push_back(state.path[i]->x);
                        state.pathVertices.push_back(state.path[i]->y);
                    }
                    updatePathBuffers(state.pathVertices);
                    state.pathFound = true;
                }
                glBindVertexArray(PathVAO);
                glDrawArrays(GL_LINES, 0, state.pathVertices.size() / 2);

                if(state.pathIndex < state.path.size()){
                    if(deltaTime >= 0.1f){
                        carVertices = getCarVertices(state.path[state.pathIndex]->x, state.path[state.pathIndex]->y, state.path[state.pathIndex]->theta, carLength, carWidth);
                        state.pathIndex++;
                        lastFrame = currentFrame;
                    } 
                    glBindBuffer(GL_ARRAY_BUFFER, carVBO);
                    glBufferData(GL_ARRAY_BUFFER, carVertices.size() * sizeof(float), carVertices.data(), GL_DYNAMIC_DRAW);
                    glUniform4f(colorLoc, 0.0f, 0.0f, 1.0f, 1.0f);
                    glBindVertexArray(carVAO);
                    glDrawElements(GL_TRIANGLES, sizeof(rectangleIndices) / sizeof(unsigned int), GL_UNSIGNED_INT, 0);
                }
            }
        } 

        if(!state.obstacles.empty()){
            glUniform4f(colorLoc, 0.5f, 0.5f, 0.5f, 1.0f);
            for(const auto& obs : state.obstacles){
                std::vector<float> obsVertices = {
                    obs.minX, obs.minY,
                    obs.maxX, obs.minY,
                    obs.maxX, obs.maxY,
                    obs.minX, obs.maxY
                };
                glBindBuffer(GL_ARRAY_BUFFER, carVBO);
                glBufferData(GL_ARRAY_BUFFER, obsVertices.size() * sizeof(float), obsVertices.data(), GL_DYNAMIC_DRAW);
                glBindVertexArray(carVAO);
                glDrawElements(GL_TRIANGLES, sizeof(rectangleIndices) / sizeof(unsigned int), GL_UNSIGNED_INT, 0);
            }
        }
        if(state.dragging){
            std::vector<float> dragVertices = {
                state.obsInitX, state.obsInitY,
                state.dragX, state.obsInitY,
                state.dragX, state.dragY,
                state.obsInitX, state.dragY
            };
            glBindBuffer(GL_ARRAY_BUFFER, carVBO);
            glBufferData(GL_ARRAY_BUFFER, dragVertices.size() * sizeof(float), dragVertices.data(), GL_DYNAMIC_DRAW);
            glUniform4f(colorLoc, 0.5f, 0.5f, 0.5f, 1.0f);
            glBindVertexArray(carVAO);
            glDrawElements(GL_TRIANGLES, sizeof(rectangleIndices) / sizeof(unsigned int), GL_UNSIGNED_INT, 0);
        }
        glUniform4f(colorLoc, 1.0f, 1.0f, 0.1f, 1.0f);
        
        if(state.startSet || state.goalSet){
            glBindBuffer(GL_ARRAY_BUFFER, pointsVBO);
            glBufferData(GL_ARRAY_BUFFER, state.initAndGoalVertices.size() * sizeof(float), state.initAndGoalVertices.data(), GL_DYNAMIC_DRAW);
        }

        glUniform4f(colorLoc, 1.0f, 1.0f, 0.1f, 1.0f);
        glBindVertexArray(pointsVAO);
        glDrawArrays(GL_POINTS, 0, state.initAndGoalVertices.size() / 2);

        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    glfwTerminate();
    return 0;
}