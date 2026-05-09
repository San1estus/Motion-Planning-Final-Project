#pragma once
#include "config.h"

extern float carLength;
extern float carWidth;
typedef struct Node;
typedef struct KDNode;

float padding = std::hypot(carLength/2, carWidth/2);

// RRT utilities
struct Node {
    float x, y, theta;
    Node* parent;
};

const float PI = acos(-1.0f);

float distance(Node* a, Node* b) {
    float angleDiff = fabs(a->theta - b->theta);
    float angleDist = std::min(angleDiff, 2 * PI - angleDiff);
    return sqrt((a->x - b->x) * (a->x - b->x) + (a->y - b->y) * (a->y - b->y) + angleDist * angleDist);
}

struct Obstacle{
    float minX, maxX, minY, maxY;
};

bool isInCollision(float x, float y, const std::vector<Obstacle>& obstacles){
    for(auto obstacle : obstacles){
        if((obstacle.minX - padding) <= x && (obstacle.maxX+padding) >= x && (obstacle.minY - padding) <= y && (obstacle.maxY+padding) >= y){
            return true;
        }
    }
    return false;
}

// RRT implementation
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

    Node* steer(Node* from, int numSteps, float stepSize, const std::vector<Obstacle>& obstacles) {
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

        return (isInCollision(newX, newY, obstacles) ? nullptr : addNode(newX, newY, newTheta, from));
        
    }

    Node* build(float goalX, float goalY, float goalRadius, int maxIter, float stepSize, float goalBias, const std::vector<Obstacle>& obstacles) {
        
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

            Node* newNode = steer(nearestNode, 10, stepSize, obstacles);
            if(newNode == nullptr){
                currIter++;
                return nullptr;
            }
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

// AppState for verifications
struct AppState {
    RRT rrt;
    Node* goalNode = nullptr;
    bool startSet = false;
    bool goalSet = false;
    bool startAlgorithm = false;
    bool obstacleMode = false;
    bool dragging = false;
    float obsInitX, obsInitY;
    float dragX, dragY;
    std::vector<Obstacle> obstacles;
    std::vector<float> initAndGoalVertices = {0.0f, 0.0f, 0.0f, 0.0f}; // startX, startY, goalX, goalY
    std::vector<Node*> path;
    std::vector<float> pathVertices;
    bool pathFound = false;
    int pathIndex = 0;
};