/*RRT Implementation for a simple car*/

#include "config.h"
#include "shader_utils.h"
#define min(a,b) ((a) < (b) ? (a) : (b))

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

unsigned int carIndices[] = {
    0, 1, 2,
    2, 3, 0
};

// Global variables for VAOs and VBOs
GLuint TreeVAO, TreeVBO;
GLuint PathVAO, PathVBO;
GLuint pointsVAO, pointsVBO;
GLuint carVAO, carVBO, carEBO;

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

    glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(carIndices), carIndices, GL_STATIC_DRAW);
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

// RRT implementation and flags
struct Node {
    float x, y, theta;
    Node* parent;
};

const float PI = acos(-1.0f);

float distance(Node* a, Node* b) {
    float angleDiff = fabs(a->theta - b->theta);
    float angleDist = min(angleDiff, 2 * PI - angleDiff);
    return sqrt((a->x - b->x) * (a->x - b->x) + (a->y - b->y) * (a->y - b->y) + angleDist * angleDist);
}

struct RRT{
    std::vector<std::unique_ptr<Node>> nodes;
    int currIter = 0;
    std::mt19937 rng;
    std::uniform_real_distribution<float> distX;
    std::uniform_real_distribution<float> distY;
    std::uniform_real_distribution<float> distBias;
    std::uniform_int_distribution<int> sampleV{-1, 1};
    std::uniform_real_distribution<float> samplePhi{-PI/4, PI/4};
    std::uniform_real_distribution<float> sampleTheta{-PI, PI};

    float distWheels = 0.5f; 
    void init(float minX, float maxX, float minY, float maxY, float startX, float startY){
        Node* init = addNode(startX, startY, 0.0f, nullptr);
        rng = std::mt19937(std::random_device{}());
        distX = std::uniform_real_distribution<float>(minX, maxX);
        distY = std::uniform_real_distribution<float>(minY, maxY);
        distBias = std::uniform_real_distribution<float>(0.0f, 1.0f);
    }

    Node* addNode(float x, float y, float theta, Node* parent) {
        auto node = std::make_unique<Node>();
        node->x = x;
        node->y = y;
        node->theta = theta;
        node->parent = parent;
        nodes.push_back(std::move(node));
        return nodes.back().get();
    }

    Node* nearest(float x, float y, float theta) {
        Node* nearestNode = nullptr;
        Node tempNode = Node{x, y, theta, nullptr};
        float minDist = std::numeric_limits<float>::max();
        for (const auto& node : nodes) {
            float dist = distance(node.get(), &tempNode);
            if (dist < minDist) {
                minDist = dist;
                nearestNode = node.get();
            }
        }

        return nearestNode;
    }

    Node* steer(Node* from, int numSteps, float stepSize) {
        float newX = from->x;
        float newY = from->y;
        float newTheta = from->theta;
        for(int i = 0; i < numSteps; ++i){
            int v = sampleV(rng);
            float phi = samplePhi(rng);
            newX += v * stepSize * cos(newTheta) * cos(phi);
            newY += v * stepSize * sin(newTheta) * cos(phi);
            newTheta += v * stepSize * tan(phi) / distWheels;
        }
        return addNode(newX, newY, newTheta, from);
        
    }

    Node* build(float goalX, float goalY, float goalRadius, int maxIter, float stepSize, float goalBias) {
        
        if(currIter < maxIter){
            float randX = distX(rng);
            float randY = distY(rng);
            float randTheta = sampleTheta(rng);
            if(distBias(rng) < goalBias){
                randX = goalX;
                randY = goalY;
                randTheta = atan2(goalY - nodes.back()->y, goalX - nodes.back()->x);
            }
            Node* nearestNode = nearest(randX, randY, randTheta);
            
            Node* newNode = steer(nearestNode, 10, stepSize);
            if(std::hypot(newNode->x - goalX, newNode->y - goalY) < goalRadius){
                return addNode(goalX, goalY, atan2(goalY - newNode->y, goalX - newNode->x), newNode);
            } 
            currIter++;
        }
        return nullptr;
    }
    
    std::vector<Node*> getPath(Node* goal){
        Node* curr = goal;
        std::vector<Node*> path;
        while(curr){
            path.push_back(curr);
            curr = curr->parent;
        }
        std::reverse(path.begin(), path.end());
        return path;
    }
    
    std::vector<float> getEdgeVertices(){
        std::vector<float> vertices;
        for(const auto& node : nodes){
            if(node->parent){
                vertices.push_back(node->parent->x);
                vertices.push_back(node->parent->y);
                vertices.push_back(node->x);
                vertices.push_back(node->y);
            }
        }
        return vertices;
    }
};

struct AppState {
    RRT rrt;
    Node* goalNode = nullptr;
    bool startSet = false;
    bool goalSet = false;
    bool startAlgorithm = false;
    std::vector<float> initAndGoalVertices = {0.0f, 0.0f, 0.0f, 0.0f}; // startX, startY, goalX, goalY
    std::vector<Node*> path;
    std::vector<float> pathVertices;
    bool pathFound = false;
    int pathIndex = 0;
};

// OpenGL callbacks
float startX, startY, goalX, goalY;
void mouseButtonCallback(GLFWwindow* window, int button, int action, int mods) {
    AppState* state = (AppState*)glfwGetWindowUserPointer(window);
    
    double xpos, ypos;
    glfwGetCursorPos(window, &xpos, &ypos);
    int width, height;
    glfwGetWindowSize(window, &width, &height);
    float xworld = (float)xpos / width * 2.0f - 1.0f;
    float yworld = 1.0f - (float)ypos / height * 2.0f;
    
    if (button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_PRESS) {
        state->initAndGoalVertices[0] = xworld;
        state->initAndGoalVertices[1] = yworld;
        state->startSet = true;
    } else if (button == GLFW_MOUSE_BUTTON_RIGHT && action == GLFW_PRESS) {
        state->initAndGoalVertices[2] = xworld;
        state->initAndGoalVertices[3] = yworld;
        state->goalSet = true;
    }
}

void keyCallback(GLFWwindow* window, int key, int scancode, int action, int mods) {
    AppState* state = (AppState*)glfwGetWindowUserPointer(window);
    
    switch (key) {
        case GLFW_KEY_R:
            if (action == GLFW_PRESS) {
                state->rrt.nodes.clear();
                state->rrt.currIter = 0;
                state->goalNode = nullptr;
                state->path.clear();
                state->pathVertices.clear();
                state->pathFound = false;
                state->pathIndex = 0;
            }
            break;
        case GLFW_KEY_SPACE:
            if (action == GLFW_PRESS) {
                if(state->startSet && state->goalSet) state->startAlgorithm = true;
                state->rrt.init(-1.0f, 1.0f, -1.0f, 1.0f, state->initAndGoalVertices[0], state->initAndGoalVertices[1] );
            }
            break;
        case GLFW_KEY_ESCAPE:
            if (action == GLFW_PRESS)
                glfwSetWindowShouldClose(window, true);
            break;
    }
}

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
    
    
    glBindBuffer(GL_ARRAY_BUFFER, pointsVBO); 
    glBufferData(GL_ARRAY_BUFFER, state.initAndGoalVertices.size() * sizeof(float), state.initAndGoalVertices.data(), GL_STATIC_DRAW);
    while(!glfwWindowShouldClose(window)) {
        
        float currentFrame = glfwGetTime();
        deltaTime = currentFrame - lastFrame;
        glClear(GL_COLOR_BUFFER_BIT);
        glUseProgram(shaderProgram);
        
        std::vector<float> edgeVertices;
            
        if(state.startAlgorithm){
            if(!state.goalNode){
                state.goalNode = state.rrt.build(state.initAndGoalVertices[2], state.initAndGoalVertices[3], 0.1f, 10000, 0.02f, 0.1f);
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
                    glDrawElements(GL_TRIANGLES, sizeof(carIndices) / sizeof(unsigned int), GL_UNSIGNED_INT, 0);
                }
            }
        }
        
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