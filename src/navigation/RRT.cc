#include "RRT.h"
#include <stdlib.h>

#include <time.h>
#include <stdio.h>
#include <vector>
#include <unordered_set>
#include <math.h>

#include <float.h>
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "gflags/gflags.h"
using std::vector;
using std::unordered_set;
#include "config_reader/config_reader.h"
#include "obstacle_avoidance/car_params.h"

namespace rrt{
    CONFIG_FLOAT(map_length_dist, "map_length_dist");
    CONFIG_FLOAT(dist_res, "dist_res");
    CONFIG_FLOAT(global_planner_curvature_num, "global_planner_curvature_num");
    CONFIG_FLOAT(global_planner_step_length, "global_planner_step_length");
    CONFIG_INT(global_planner_per_step_num, "global_planner_per_step_num");
    CONFIG_INT(global_planner_steps, "global_planner_steps");
    int dirs[8][2] = {
        {1, 0}, {-1, 0}, {0, 1}, {0, -1},
        // {0, 0}, {0, 0}, {0, 0}, {0, 0}    
        {1, -1}, {1, 1}, {-1, 1}, {-1, -1}    
    };
    const bool Node::operator == (const Node& node) const {
        return (this->xIdx == node.xIdx) && (this->yIdx == node.yIdx);
    }
    /*bool operator == (rrt::Node const& n1, rrt::Node const& n2)
    {
        return (n1.xIdx == n2.xIdx) && (n1.yIdx == n2.yIdx);
    }
    bool operator != (rrt::Node const& n1,rrt::Node const& n2)
    {
        if(n1.xIdx != n2.xIdx) return true;
        if(n1.yIdx != n2.yIdx) return true;
        return false;
    }*/
    bool IsNodeCollide(costmap::CostMap const& collision_map, const rrt::Node& node) {
        try {
            if(collision_map.GetValueAtIdx(node.xIdx, node.yIdx) == 1.0) return true;
        } catch(std::out_of_range) {
            return true;
        }
        return false;
    }
    bool IsNodeAndNeighbourCollide(costmap::CostMap const& collision_map, Eigen::Vector2f& node) {
        int xIdx = collision_map.GetIndexFromDist(node.x());
        int yIdx = collision_map.GetIndexFromDist(node.y());
        try {
            if(collision_map.GetValueAtIdx(xIdx, yIdx) == 1.0) return true;
            for(int i = 0; i < 8; i++) {
                int newXIdx = xIdx + dirs[i][0];
                int newYIdx = yIdx + dirs[i][1];
                if(collision_map.GetValueAtIdx(newXIdx, newYIdx) == 1.0) return true;
            } 
        } catch(std::out_of_range) {
            return true;
        }
        return false;
    }
    int RRT(costmap::CostMap const& collision_map, rrt::Node& start, rrt::Node& end, amrl_msgs::VisualizationMsg& msg) {
        start.xIdx = collision_map.GetIndexFromDist(start.x);
        start.yIdx = collision_map.GetIndexFromDist(start.y);
        end.xIdx = collision_map.GetIndexFromDist(end.x);
        end.yIdx = collision_map.GetIndexFromDist(end.y);
        int xSize = collision_map.GetRowNum();
        int ySize = collision_map.GetColNum();
        list<const rrt::Node *> plan;

        vector<rrt::Node> graph;
        unordered_set<rrt::Node, rrt::NodeHash> nodeSet;

        graph.push_back(start);
        nodeSet.insert(start);
        int cnt = 0;
        int lim = CONFIG_global_planner_steps;
        visualization::DrawCross(Eigen::Vector2f(start.x, start.y), 0.25,0xfc4103,msg);
        visualization::DrawCross(Eigen::Vector2f(end.x, end.y), 0.25,0xfc4103,msg);
        while(cnt < lim) {
            cnt++;
            rrt::Node newNode = CreateNewNode(xSize, ySize);
            
            // CurStepOutliner(newNode, msg);
            // std::cout<<"Generated newNode x:" << newNode.x << "Generated newNode y:" << newNode.y << "\n";
            if(IsNodeCollide(collision_map, newNode)) continue;
            rrt::Node* nearestNode = FindNearestNode(collision_map, newNode, end, graph); 
            if(nearestNode == NULL ||( (nearestNode->xIdx) == newNode.xIdx && (nearestNode->yIdx) == newNode.yIdx)) continue;
            newNode.parent = nearestNode;
            if(nodeSet.find(newNode) == nodeSet.end()) {
                
                graph.push_back(newNode);
                nodeSet.insert(newNode);
            }
            
            CurStepOutliner(newNode, msg);
            // std::cout<<"nearestNode x:" << nearestNode->x << " nearestNode y:" << nearestNode->y << "\n";
            // std::cout<<"newNode x:" << newNode.x << " newNode y:" << newNode.y << "\n";
            /* 
            if(abs(nearestNode->angle) < 1e-5) {
                visualization::DrawLine(Eigen::Vector2f(nearestNode->x, nearestNode->y), Eigen::Vector2f(newNode.x, newNode.y),0x1e9aa8,msg);
            } else {
                Eigen::Vector2f center = GetCenter(*nearestNode, nearestNode->curvature);
                visualization::DrawArc(center, 1/nearestNode->curvature, M_PI/2 + nearestNode->angle, M_PI/2 + newNode.angle,0x1e9aa8,msg);
            }*/
            if(newNode.xIdx == end.xIdx && newNode.yIdx == end.yIdx) {
                // ConstructPath(start, end, plan);
                return 1;
            }
        }
        return 0;
    }
    rrt::Node CreateNewNode(int xSize, int ySize) {
        // srand (time(NULL));
        int xIdx = rand() % xSize;
        int yIdx = rand() % ySize;
        // int xIdx = 5;
        // int yIdx = 5;
        rrt::Node newNode(xIdx, yIdx);
        newNode.x = -CONFIG_map_length_dist + newNode.xIdx * CONFIG_dist_res;
        newNode.y = -CONFIG_map_length_dist + newNode.yIdx * CONFIG_dist_res;
        // newNode.x = (xIdx + 0.5) * CONFIG_dist_res;
        // newNode.y = (yIdx + 0.5) * CONFIG_dist_res;
        return newNode;
    }
    rrt::Node* FindNearestNode(costmap::CostMap const& collision_map, rrt::Node& newNode, rrt::Node const& end, vector<rrt::Node>& graph) {
        rrt::Node* nearestNode = NULL;
        double minDist = DBL_MAX;
        for(rrt::Node& n: graph) {
            double curDist = GetGridDist(newNode, n);
            if(curDist < minDist) {
                nearestNode = &n;
                minDist = curDist;
            }
        }
        // TODO: control, change newNode
        double curvature_inc = (car_params::max_curvature - car_params::min_curvature) / CONFIG_global_planner_curvature_num;
        double minGridDist = DBL_MAX;
        rrt::Node bestNextStep;
        for(float curvature = car_params::min_curvature; curvature < car_params::max_curvature; curvature += curvature_inc) {
            Eigen::Vector2f center = GetCenter(*nearestNode, curvature);            
            rrt::Node step_end;
            if(CheckCollision(collision_map, *nearestNode, end, center, curvature, step_end)) {
                continue;
            } else {
                float curGridDist = GetDist(step_end, newNode);
                if(curGridDist < minGridDist) {
                    minGridDist = curGridDist;
                    bestNextStep = step_end;
                    (*nearestNode).curvature = curvature;
                }
            }
        }
        if(minGridDist == DBL_MAX) return NULL;
        newNode = bestNextStep;
        newNode.xIdx = collision_map.GetIndexFromDist(newNode.x);
        newNode.yIdx = collision_map.GetIndexFromDist(newNode.y);
        return nearestNode;
    }
    void ConstructPath(const rrt::Node& start, const rrt::Node& end, list<rrt::Node const* > plan) {
        const rrt::Node* cur = &end;
        do {
            plan.push_front(cur);
            cur = cur->parent;
        } while(((cur->xIdx) != start.xIdx) || (cur->yIdx != start.yIdx));
    }
    Eigen::Vector2f GetCenter(rrt::Node const& node, float curvature) {
        if(abs(curvature) < 1e-5) {
            return Eigen::Vector2f();
        }
        float centerX = node.x - 1.0 / curvature * sin(node.angle);
        float centerY = node.y + 1.0 / curvature * cos(node.angle);
        Eigen::Vector2f center = {centerX, centerY};
        return center;
    }
    double GetStepEndAngle(rrt::Node const & step_end, Eigen::Vector2f& center, float curDelta) {
        if(curDelta < 0) return atan2(center.x() - step_end.x, step_end.y - center.y());
        else return atan2(step_end.x - center.x(), center.y() - step_end.y);
    }
    bool CheckCollision(costmap::CostMap const& collision_map, rrt::Node const& curNode, rrt::Node const& end, Eigen::Vector2f& center, float curvature, rrt::Node& step_end) {
        double curDelta;
        if(abs(curvature) < 1e-5) {
            for(int i = CONFIG_global_planner_per_step_num; i >= 1 ; i--) {
                curDelta = CONFIG_global_planner_step_length / M_PI / i;
                Eigen::Vector2f curLoc = {curNode.x + cos(curNode.angle) * curDelta, curNode.y + sin(curNode.angle) * curDelta};
                if(IsNodeAndNeighbourCollide(collision_map, curLoc)) {
                    return true;
                }
                step_end.x = curLoc.x();
                step_end.y = curLoc.y();
                step_end.angle = curNode.angle;
                if(end.xIdx == step_end.xIdx && end.yIdx == step_end.yIdx ) {
                    return false;
                }
            }
        } else {
            for(int i = CONFIG_global_planner_per_step_num; i >= 1 ; i--) {
                curDelta = CONFIG_global_planner_step_length / M_PI / i;
                if(curvature < 0) curDelta = -curDelta;
                Eigen::Vector2f center2node = {curNode.x - center.x(), curNode.y - center.y()};
                Eigen::Vector2f curLoc = Eigen::Rotation2Df(curDelta) * (center2node) + center;
                if(IsNodeAndNeighbourCollide(collision_map, curLoc)) {
                    return true;
                }
                step_end.x = curLoc.x();
                step_end.y = curLoc.y();
                step_end.angle = GetStepEndAngle(step_end, center, curDelta);
                if(end.xIdx == step_end.xIdx && end.yIdx == step_end.yIdx) {
                    return false;
                }
            }
        }
        return false;
    }
    
    double GetGridDist(rrt::Node const& n1, rrt::Node const& n2) {
        double curDist = sqrt(pow(n1.xIdx - n2.xIdx, 2) + pow(n1.yIdx - n2.yIdx, 2));
        return curDist;
    }
    double GetDist(rrt::Node const& n1, rrt::Node const& n2) {
        double curDist = sqrt(pow(n1.x - n2.x, 2.0) + pow(n1.y - n2.y, 2.0));
        return curDist;
    }
    void CurStepOutliner(rrt::Node const& node, amrl_msgs::VisualizationMsg& msg) {
        visualization::DrawCross(Eigen::Vector2f(node.x, node.y), 0.25,0x1e9aa8,msg);
    }
    // Draw the global path
    void GlobalPathOutliner(list<rrt::Node*>& plan, amrl_msgs::VisualizationMsg& msg){
        rrt::Node* prev = NULL;
        for(rrt::Node* n: plan) {
            if(prev != NULL) {
                if(abs(prev->angle) < 1e-5) {
                    visualization::DrawLine(Eigen::Vector2f(prev->x, prev->y), Eigen::Vector2f(n->x, n->y),0x1e9aa8,msg);
                } else {
                    Eigen::Vector2f center = GetCenter(*prev, prev->curvature);
                    visualization::DrawArc(center, 1/prev->curvature, M_PI/2 + prev->angle, M_PI/2 + n->angle,0x1e9aa8,msg);
                }
            }
            CurStepOutliner(*n, msg);
            prev = n;
        } 
    }

    
}
