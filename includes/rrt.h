#pragma once
#include "config.h"

extern float carLength;
extern float carWidth;
typedef struct Node;
typedef struct KDNode;

// RRT utilities
float padding = std::hypot(carLength/2, carWidth/2);

struct Node {
    float x, y, theta, cost;
    Node* parent;
};

const float PI = acos(-1.0f);

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
bool edgeCollision(Node* a, Node* b, const std::vector<Obstacle>& obstacles){
    int samples = 10;
    for(int i = 0; i <= samples; i++){
        float t = (float)i/samples;
        float x = a->x + t * (b->x - a->x);
        float y = a->y + t * (b->y - a->y);
        if(isInCollision(x, y, obstacles)) return true;
    }
    
    return false;
}
float distance(Node* a, Node* b) {
    float angleDiff = fabs(a->theta - b->theta);
    float angleDist = std::min(angleDiff, 2 * PI - angleDiff);
    return sqrt((a->x - b->x) * (a->x - b->x) + (a->y - b->y) * (a->y - b->y) + angleDist * angleDist);
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

        KDNode* insertRecursive(KDNode* node, Node* point, int depth){
            if (node == nullptr) return new KDNode(point);
            int currDim = depth % 2;
            if (!currDim){ // X Axis
                if(point->x < node->point->x){
                    node->left = insertRecursive(node->left, point, depth+1);
                } else{
                    node->right = insertRecursive(node->right, point, depth+1);
                }
            }
            else{// Y Axis
                if(point->y < node->point->y){
                    node->left = insertRecursive(node->left, point, depth+1);
                } else{
                    node->right = insertRecursive(node->right, point, depth+1);
                }
            }

            return node;
        }

        bool searchRecursive(KDNode* node, Node* point, int depth) const {
            if (node == nullptr) return false;

            if (node->point == point) return true;
            int currDim = depth % 2;
            if (!currDim){ // X Axis
                if(point->x < node->point->x){
                    return searchRecursive(node->left, point, depth+1);
                } else{
                    return searchRecursive(node->right, point, depth+1);
                }
            }
            else{// Y Axis
                if(point->y < node->point->y){
                    return searchRecursive(node->left, point, depth+1);
                } else{
                    return searchRecursive(node->right, point, depth+1);
                }
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
            int currDim = depth % 2;
            if(!currDim){
                if(node->left != nullptr && query->x - bestDist <= point->x){
                    nearestRecursive(node->left, query, depth+1, best, bestDist);
                }
                if(node->right != nullptr && query->x <= point->x + bestDist){
                    nearestRecursive(node->right, query, depth+1, best, bestDist);
                } 
            } else{
                if(node->left != nullptr && query->y - bestDist <= point->y){
                    nearestRecursive(node->left, query, depth+1, best, bestDist);
                }
                if(node->right != nullptr && query->y <= point->y + bestDist){
                    nearestRecursive(node->right, query, depth+1, best, bestDist);
                } 
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
            int currDim = depth % 2;
            if(!currDim){
                if(node->left != nullptr && query->x - r <= point->x){
                    nearRecursive(node->left, query, r, depth+1, result);
                }
                if(node->right != nullptr && query->x + r >= point->x){
                    nearRecursive(node->right, query, r, depth+1, result);
                } 
            } else{
                if(node->left != nullptr && query->y - r <= point->y){
                    nearRecursive(node->left, query, r, depth+1, result);
                }
                if(node->right != nullptr && query->y + r >= point->y){
                    nearRecursive(node->right, query, r, depth+1, result);
                } 
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


// RRT implementation
struct RRT{
    std::vector<std::unique_ptr<Node>> nodes;
    int currIter = 0;
    std::mt19937 rng;
    std::uniform_real_distribution<float> distX;
    std::uniform_real_distribution<float> distY;
    std::uniform_real_distribution<float> distBias;
    std::uniform_int_distribution<int> sampleV{0, 1};
    std::uniform_real_distribution<float> samplePhi{-PI/8, PI/8};
    std::uniform_real_distribution<float> sampleTheta{-PI, PI};
    KDTree kdTree;
    float gammaRRT;
    float distWheels = 0.5f; 

    void init(float minX, float maxX, float minY, float maxY, float startX, float startY, const std::vector<Obstacle> obstacles){
        Node* init = addNode(startX, startY, sampleTheta(rng), nullptr);
        rng = std::mt19937(std::random_device{}());
        distX = std::uniform_real_distribution<float>(minX, maxX);
        distY = std::uniform_real_distribution<float>(minY, maxY);
        distBias = std::uniform_real_distribution<float>(0.0f, 1.0f);
        float obstacleArea = 0.0f;
        for(auto obstacle : obstacles){
            obstacleArea += (obstacle.maxX-obstacle.minX) * (obstacle.maxY-obstacle.minY);
        }
        float muFree = (4.0f - obstacleArea)/4.0f;
        gammaRRT = sqrt(3)*sqrt(muFree/PI);
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

    Node* steer(Node* from, int numSteps, float stepSize, const std::vector<Obstacle>& obstacles) {
        float newX = from->x;
        float newY = from->y;
        float newTheta = from->theta;
        float v = sampleV(rng) ? 1.0f : -1.0f;
        float phi = samplePhi(rng);
        for(int i = 0; i < numSteps; ++i){
            newX += v * stepSize * cos(newTheta) * cos(phi);
            newY += v * stepSize * sin(newTheta) * cos(phi);
            if(newX > 0.95 || newX < -0.95 || newY < -0.95 || newY > 0.95) break;
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

            Node* newNode = steer(nearestNode, 5, stepSize, obstacles);
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

    Node* buildStar(float goalX, float goalY, float goalRadius, int maxIter, float stepSize, float goalBias, const std::vector<Obstacle>& obstacles) {
        
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

            Node* newNode = steer(nearestNode, 30, stepSize*0.5f, obstacles);
            if(newNode == nullptr){
                currIter++;
                return nullptr;
            }
            
            size_t n = nodes.size();
            float r_n = gammaRRT*sqrt(log(n)/n);
            Node* minNode = nearestNode;
            float minCost = minNode->cost + distance(minNode, newNode);
            std::vector<Node*> xNear = kdTree.near(newNode, r_n);
            for(auto nearNode : xNear){
                float nearCost = nearNode->cost + distance(newNode, nearNode);
                if(!edgeCollision(nearNode, newNode, obstacles) && nearCost < minCost){
                    minNode = nearNode;
                    minCost = nearCost;
                }
            }
            newNode->parent = minNode;
            newNode->cost = minCost;
            for(auto nearNode: xNear){
                float nearCost = newNode->cost + distance(newNode, nearNode);
                if(!edgeCollision(nearNode, newNode, obstacles) && nearCost < nearNode->cost){
                    nearNode->parent = newNode;
                    nearNode->cost  = nearCost;
                }
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