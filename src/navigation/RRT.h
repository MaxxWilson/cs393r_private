#ifndef RRT_H
#define RRT_H
#include <vector>
#include <list>
#include "shared/math/geometry.h"
#include "cost_map/cost_map.h"
#include "visualization/visualization.h"
using geometry::line2f;
using std::vector;
using std::list;
using costmap::CostMap;
namespace rrt{
class Node {
public:
    // Rasterized map Idx
    int xIdx;
    int yIdx;
    // map frame coordinate
    float x;
    float y;
    float angle; // current angle (-PI -> PI)
    float curvature; // for visualization
    Node* parent;
    Node(): xIdx(-1), yIdx(-1), parent(NULL) {}
    Node(float _x, float _y, float _angle): x(_x), y(_y), angle(_angle) {}
    Node(int _xIdx, int _yIdx): xIdx(_xIdx), yIdx(_yIdx), parent(NULL) {}
    Node(int _xIdx, int _yIdx, float _angle): xIdx(_xIdx), yIdx(_yIdx), angle(_angle), parent(NULL) {}
    // friend bool operator == (Node const &, Node const &);
    // friend bool operator != (Node const &, Node const &);
    const bool operator == (const Node& node) const;
};

struct NodeHash {
    std::size_t operator() (const Node& t) const {
        return t.x * 100 + t.y;
    }
};
// 
rrt::Node CreateNewNode(int xSize, int ySize);
int RRT(costmap::CostMap const& collision_map, Node& start, Node& end, amrl_msgs::VisualizationMsg& msg);
bool IsInObstacle(costmap::CostMap& collision_map, const Node& node);
double GetGridDist(rrt::Node const& n1, rrt::Node const& n2);
rrt::Node* FindNearestNode(costmap::CostMap const& collision_map, rrt::Node & newNode, rrt::Node const& end, vector<rrt::Node>& graph);

void CurStepOutliner(rrt::Node const& node, amrl_msgs::VisualizationMsg& msg);
Eigen::Vector2f GetCenter(rrt::Node const& node, float curvature);
double GetDist(rrt::Node const& n1, rrt::Node const& n2);
void ConstructPath(const rrt::Node& start, const rrt::Node& end, list<rrt::Node * > plan);
bool CheckCollision(costmap::CostMap const& collision_map, rrt::Node const& curNode, rrt::Node const& end, Eigen::Vector2f& center, float curvature, rrt::Node& step_end);
}


#endif // RRT_H