/*First RRT Implementation */

#include "config.h"
#include "shader_utils.h"

GLuint TreeVAO, TreeVBO;
GLuint PathVAO, PathVBO;
GLuint pointsVAO, pointsVBO;
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
}

void updateTreeBuffers(const std::vector<float>& vertices) {
    glBindBuffer(GL_ARRAY_BUFFER, TreeVBO);
    glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(float), vertices.data(), GL_DYNAMIC_DRAW);
}

void updatePathBuffers(const std::vector<float>& vertices) {
    glBindBuffer(GL_ARRAY_BUFFER, PathVBO);
    glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(float), vertices.data(), GL_DYNAMIC_DRAW);
}


struct Node {
    float x, y;
    Node* parent;
};

struct RRT{
    std::vector<std::unique_ptr<Node>> nodes;
    int currIter = 0;
    std::mt19937 rng;
    std::uniform_real_distribution<float> distX;
    std::uniform_real_distribution<float> distY;
    std::uniform_real_distribution<float> distBias;
    
    void init(float minX, float maxX, float minY, float maxY){
        Node* init = addNode(0.0f, 0.0f, NULL);
        rng = std::mt19937(std::random_device{}());
        distX = std::uniform_real_distribution<float>(minX, maxX);
        distY = std::uniform_real_distribution<float>(minY, maxY);
        distBias = std::uniform_real_distribution<float>(0.0f, 1.0f);
    }

    Node* addNode(float x, float y, Node* parent) {
        auto node = std::make_unique<Node>();
        node->x = x;
        node->y = y;
        node->parent = parent;
        nodes.push_back(std::move(node));
        return nodes.back().get();
    }

    Node* nearest(float x, float y) {
        Node* nearestNode = nullptr;
        float minDist = std::numeric_limits<float>::max();
        for (const auto& node : nodes) {
            float dist = std::hypot(node->x - x, node->y - y);
            if (dist < minDist) {
                minDist = dist;
                nearestNode = node.get();
            }
        }
        return nearestNode;
    }

    Node* steer(Node* from, float toX, float toY, float stepSize) {
        float dx = toX - from->x;
        float dy = toY - from->y;
        float dist = std::hypot(dx, dy);
        if (dist < stepSize) {
            return addNode(toX, toY, from);
        } else {
            float newX = from->x + (dx / dist) * stepSize;
            float newY = from->y + (dy / dist) * stepSize;
            return addNode(newX, newY, from);
        }
    }

    Node* build(float goalX, float goalY, float goalRadius, int maxIter, float stepSize, float goalBias) {
        
        if(currIter < maxIter){
            float randX = distX(rng);
            float randY = distY(rng);
            if(distBias(rng) < goalBias){
                randX = goalX;
                randY = goalY;
            }
            Node* nearestNode = nearest(randX, randY);
            
            Node* newNode = steer(nearestNode, randX, randY, stepSize);
            if(std::hypot(newNode->x - goalX, newNode->y - goalY) < goalRadius){
                return addNode(goalX, goalY, newNode);
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
    GLuint shaderProgram = createShaderProgram(vertSrc.c_str(), fragSrc.c_str());;
    GLint colorLoc = glGetUniformLocation(shaderProgram, "uColor");
    
    RRT rrt;
    rrt.init(-1.0f, 1.0f, -1.0f, 1.0f);
    Node* goalNode = nullptr;
    std::vector<Node*> path;
    std::vector<float> pathVertices;
    std::vector<float> initAndGoalVertices = {0.0f, 0.0f, 0.8f, 0.8f};
    
    bool pathFound = false;
    setupBuffers();
    glBufferData(GL_ARRAY_BUFFER, initAndGoalVertices.size() * sizeof(float), initAndGoalVertices.data(), GL_STATIC_DRAW);
    glPointSize(10.0f);
    while(!glfwWindowShouldClose(window)) {
        glClear(GL_COLOR_BUFFER_BIT);
        glUseProgram(shaderProgram);
        
        std::vector<float> edgeVertices;
            if(!goalNode){
                goalNode = rrt.build(initAndGoalVertices[2], initAndGoalVertices[3], 0.05f, 1000, 0.05f, 0.1f);
                if(goalNode){
                    path = rrt.getPath(goalNode);
                }
                edgeVertices = rrt.getEdgeVertices();
                updateTreeBuffers(edgeVertices);
            } else{
                edgeVertices = rrt.getEdgeVertices();
            }
        
        glUniform4f(colorLoc, 1.0f, 0.0f, 0.0f, 1.0f);
        glBindVertexArray(TreeVAO);
        glDrawArrays(GL_LINES, 0, edgeVertices.size() / 2);

        if(!path.empty()){
            glUniform4f(colorLoc, 0.0f, 1.0f, 0.0f, 1.0f);
            if(!pathFound){
                for(size_t i = 1; i < path.size(); ++i){
                    pathVertices.push_back(path[i-1]->x);
                    pathVertices.push_back(path[i-1]->y);
                    pathVertices.push_back(path[i]->x);
                    pathVertices.push_back(path[i]->y);
                }
                updatePathBuffers(pathVertices);
                pathFound = true;
            }
            glBindVertexArray(PathVAO);
            glDrawArrays(GL_LINES, 0, pathVertices.size() / 2);
        }
        
        glUniform4f(colorLoc, 1.0f, 1.0f, 0.1f, 1.0f);
        glBindVertexArray(pointsVAO);
        glDrawArrays(GL_POINTS, 0, initAndGoalVertices.size() / 2);

        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    glfwTerminate();
    return 0;
}