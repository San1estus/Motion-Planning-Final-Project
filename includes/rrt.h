#pragma once
#include "config.h"

// TODO: Add Catmull-Rom spline for smoother result path visualization
extern float carLength;
extern float carWidth;
struct Node;
struct KDNode;

float aspect = 1920.0f / 1080.0f;
// RRT utilities
float padding = std::hypot(carLength/2, carWidth/2);

struct Node {
    float x, y, theta, cost;
    Node* parent;
};

const float PI = acos(-1.0f);
const float maxPhi = PI/4.0f; // Max steering angle
struct Obstacle{
    float minX, maxX, minY, maxY;
};

float distance(Node* a, Node* b) {
    float angleDiff = fabs(a->theta - b->theta);
    float angleDist = std::min(angleDiff, 2 * PI - angleDiff);
    return sqrt((a->x - b->x) * (a->x - b->x) + (a->y - b->y) * (a->y - b->y) + angleDist * angleDist);
}

float normalizeAngle(float angle){
    return atan2(sin(angle), cos(angle));
}

bool isInCollision(float x, float y, const std::vector<Obstacle>& obstacles){
    for(auto obstacle : obstacles){
        if((obstacle.minX - padding) <= x && (obstacle.maxX+padding) >= x && (obstacle.minY - padding) <= y && (obstacle.maxY+padding) >= y){
            return true;
        }
    }
    return false;
}

bool edgeCollision(Node* a, Node* b, const std::vector<Obstacle>& obstacles, float distWheels = 0.15f){
    int samples = 10;
    float newX = a->x, newY = a->y, newTheta = a->theta;
    float stepSize = distance(a, b) / samples;
    for(int i = 0; i <= samples; i++){
        float desiredTheta = atan2(b->y - newY, b->x - newX);
        float thetaDiff = normalizeAngle(desiredTheta - newTheta);
        float phi = std::clamp(thetaDiff,-maxPhi, maxPhi);

        // Steer the car
        newX += stepSize * cos(newTheta);
        newY += stepSize * sin(newTheta);
        newTheta += stepSize * tan(phi) / distWheels;
        newTheta = normalizeAngle(newTheta);
        if(isInCollision(newX, newY, obstacles)) return true;
    }
    return false;
}

// KD-Tree utilities
struct KDNode{
    Node* point;
    KDNode* left;
    KDNode* right;
    KDNode(Node* node) : point(node), left(nullptr), right(nullptr){}
};

// KD-Tree implementation
class KDTree{
    private:
        KDNode* root;
        float getCoord(Node* node, int dim) {
            if(dim == 0) return node->x;
            if(dim == 1) return node->y;
            return node->theta * (aspect / PI); // Scale theta to be comparable to x and y
        }
        KDNode* insertRecursive(KDNode* node, Node* point, int depth){
            if (node == nullptr) return new KDNode(point);
            int currDim = depth % 3;
            if(getCoord(point, currDim) < getCoord(node->point, currDim)){
                node->left = insertRecursive(node->left, point, depth+1);
            } else{
                node->right = insertRecursive(node->right, point, depth+1);
            }

            return node;
        }

        bool searchRecursive(KDNode* node, Node* point, int depth){
            if (node == nullptr) return false;

            if (node->point == point) return true;
            int currDim = depth % 3;
            if(getCoord(point, currDim) < getCoord(node->point, currDim)){
                    return searchRecursive(node->left, point, depth+1);
                } else{
                    return searchRecursive(node->right, point, depth+1);
                }
        }
        
        void nearestRecursive(KDNode* node, Node* query, int depth, Node*& best, float& bestDist){
            if (node == nullptr) return;
            
            Node* point = node->point;
            float dist = distance(point, query);
            if(dist < bestDist){
                best = point;
                bestDist = dist;
            }
            int currDim = depth % 3;
            if(node->left != nullptr && getCoord(query, currDim) - bestDist <= getCoord(point, currDim)){
                nearestRecursive(node->left, query, depth+1, best, bestDist);
            }
            if(node->right != nullptr && getCoord(query, currDim) <= getCoord(point, currDim) + bestDist){
                nearestRecursive(node->right, query, depth+1, best, bestDist);
            } 
        }
        void freeTree(KDNode* node) {
            if (!node) return;
            freeTree(node->left);
            freeTree(node->right);
            delete node;
        }
        void nearRecursive(KDNode* node, Node* query, float r, int depth, std::vector<Node*>& result){
            if(node == nullptr) return;
            Node* point = node->point;
            if(distance(point, query) <= r){
                result.push_back(point);
            }
            int currDim = depth % 3;
            if(node->left != nullptr && getCoord(query, currDim) - r <= getCoord(point, currDim)){
                nearRecursive(node->left, query, r, depth+1, result);
            }
            if(node->right != nullptr && getCoord(query, currDim) + r >= getCoord(point, currDim)){
                nearRecursive(node->right, query, r, depth+1, result);
            }
        }
    public:
        KDTree() : root(nullptr){}
        ~KDTree() {freeTree(root);}
        void clear(){
            freeTree(root);
            root = nullptr;
        }
        void insert(Node* query){
            root = insertRecursive(root, query, 0);
        }
        bool search(Node* query){
            return searchRecursive(root, query, 0);
        }
        Node* nearest(Node* query){
            Node* best = nullptr;
            float bestDist = std::numeric_limits<float>::max();
            nearestRecursive(root, query, 0, best, bestDist);
            return best;
        }
        
        std::vector<Node*> near(Node* query, float r){
            std::vector<Node*> result;
            nearRecursive(root, query, r, 0, result);
            return result;
        }
};

// Steering simulation
struct SteerResult{
    bool valid;
    float x, y, theta;
};

SteerResult simulateSteer(Node* from, float targetX, float targetY, int numSteps, float stepSize, const std::vector<Obstacle>& obstacles, float distWheels = 0.15f){
    float newX = from->x;
    float newY = from->y;
    float newTheta = from->theta;

    const float threshold = PI / 9.0f;
    float prevX = newX, prevY = newY, prevTheta = newTheta;
    for(int i = 0; i < numSteps; i++){
        float desiredTheta = atan2(targetY - newY, targetX - newX);

        float thetaDiff = normalizeAngle(desiredTheta - newTheta);

        float phi;

        if(thetaDiff < -threshold)
            phi = -maxPhi;
        else if(thetaDiff > threshold)
            phi = maxPhi;
        else
            phi = 0.0f;

        newX += stepSize * cos(newTheta);
        newY += stepSize * sin(newTheta);

        newTheta += stepSize * tan(phi) / distWheels;

        newTheta = normalizeAngle(newTheta);

        if(newX > aspect || newX < -aspect ||
           newY > 0.99f || newY < -0.99f)
        {
            return {false,prevX, prevY, prevTheta};
        }

        if(isInCollision(newX, newY, obstacles))
            return {false,prevX, prevY, prevTheta};

        if(std::hypot(targetX - newX, targetY - newY) < 0.01f)
        {
            break;
        }
        prevX = newX;
        prevY = newY;
        prevTheta = newTheta;
    }

    return {true, newX, newY, newTheta};
}

// RRT implementation
struct RRT{
    std::vector<std::unique_ptr<Node>> nodes;
    int currIter = 0;
    std::mt19937 rng;
    std::uniform_real_distribution<float> distX;
    std::uniform_real_distribution<float> distY;
    std::uniform_real_distribution<float> distBias;
    std::uniform_real_distribution<float> samplePhi{-maxPhi, maxPhi};
    std::uniform_real_distribution<float> sampleTheta{-PI, PI};
    KDTree kdTree;
    float gammaRRT;
    float distWheels = 0.15f; 

    void init(float minX, float maxX, float minY, float maxY, float startX, float startY, float goalX, float goalY, const std::vector<Obstacle> obstacles){
        Node* init = addNode(startX, startY, atan2(goalY - startY, goalX - startX), nullptr);
        rng = std::mt19937(std::random_device{}());
        distX = std::uniform_real_distribution<float>(minX, maxX);
        distY = std::uniform_real_distribution<float>(minY, maxY);
        distBias = std::uniform_real_distribution<float>(0.0f, 1.0f);
        float obstacleArea = 0.0f;
        for(auto obstacle : obstacles){
            obstacleArea += (obstacle.maxX-obstacle.minX) * (obstacle.maxY-obstacle.minY);
        }
        float muFree = (4.0f*aspect - obstacleArea)/(4.0f*aspect);
        gammaRRT = cbrtf(3)*cbrtf(muFree/PI);
    }

    Node* addNode(float x, float y, float theta, Node* parent) {
        auto node = std::make_unique<Node>();
        node->x = x;
        node->y = y;
        node->theta = theta;
        node->parent = parent;
        node->cost = parent ?  parent->cost + distance(parent, node.get()) : 0.0f;
        nodes.push_back(std::move(node));
        kdTree.insert(nodes.back().get());
        return nodes.back().get();
    }

    Node* nearest(float x, float y, float theta) {
        Node query = Node{x, y, theta, 0, nullptr};
        return kdTree.nearest(&query);
    }

    Node* steer(Node* from, float targetX, float targetY, int numSteps, float stepSize, const std::vector<Obstacle>& obstacles)
    {
        auto result = simulateSteer(from, targetX, targetY, numSteps, stepSize, obstacles, distWheels);
        if(!result.valid) return nullptr;
        return addNode(result.x, result.y, result.theta, from);
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

            Node* newNode = steer(nearestNode, randX, randY, 20, 0.025f, obstacles);
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

    Node* buildStar(float goalX, float goalY, float goalRadius, int maxIter, float stepSize, float goalBias, const std::vector<Obstacle>& obstacles){
        if(currIter >= maxIter)
            return nullptr;

        // Sample
        float randX = distX(rng);
        float randY = distY(rng);
        float randTheta = sampleTheta(rng);

        if(distBias(rng) < goalBias){
            randX = goalX;
            randY = goalY;
            randTheta = atan2(goalY - nodes.back()->y, goalX - nodes.back()->x);
        }

        // Nearest
        Node* nearestNode = nearest(randX, randY, randTheta);

        // Initial extension
        Node* newNode = steer(nearestNode, randX, randY, 20, 0.0025f, obstacles);

        if(newNode == nullptr){
            currIter++;
            return nullptr;
        }

        // Near set
        size_t n = nodes.size();
        float r_n = gammaRRT * cbrtf(log((float)n)/(float)n);
        r_n = std::min(r_n, 0.05f);
        std::vector<Node*> xNear = kdTree.near(newNode, r_n);

        // Choose best parent
        Node* bestParent = nearestNode;

        float bestCost = nearestNode->cost + distance(nearestNode, newNode);

        for(auto nearNode : xNear){
            auto result = simulateSteer(nearNode, randX, randY, 20, 0.0025f, obstacles, distWheels);

            if(!result.valid)
                continue;

            Node tempNode{ result.x, result.y, result.theta, 0, nullptr};

            float candidateCost = nearNode->cost + distance(nearNode, &tempNode);

            if(candidateCost < bestCost){
                bestCost = candidateCost;

                bestParent = nearNode;
            }
        }

        // Rebuild node from best parent
        auto bestResult = simulateSteer(bestParent, randX, randY, 20, 0.0025f, obstacles, distWheels);

        if(bestResult.valid){
            newNode->x = bestResult.x;
            newNode->y = bestResult.y;
            newNode->theta = bestResult.theta;
            newNode->parent = bestParent;
            newNode->cost = bestParent->cost + distance(bestParent,  newNode);
        }

        // Rewiring
        for(auto nearNode : xNear){
            if(nearNode == newNode)
                continue;

            float newCost = newNode->cost + distance(newNode, nearNode);

            if(newCost >= nearNode->cost)
                continue;

            auto rewired = simulateSteer(newNode, nearNode->x, nearNode->y, 20, 0.0025f, obstacles, distWheels);

            if(!rewired.valid)
                continue;

            nearNode->parent = newNode;

            nearNode->cost = newCost;
        }

        // Goal check
        if(std::hypot(newNode->x - goalX,newNode->y - goalY) < goalRadius)
        {
            currIter++;

            return addNode(goalX, goalY, atan2(goalY - newNode->y,  goalX - newNode->x), newNode);
        }
        
        currIter++;
        
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