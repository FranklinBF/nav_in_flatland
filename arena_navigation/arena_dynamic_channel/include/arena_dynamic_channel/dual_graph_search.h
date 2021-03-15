
#ifndef _DUAL_GRAPH_SEARCH_H
#define _DUAL_GRAPH_SEARCH_H


#include <iostream>
#include <string>
#include <ros/console.h>
#include <ros/ros.h>

#include <Eigen/Eigen>
#include <memory>
#include <map>
#include <unordered_map>
#include <boost/functional/hash.hpp>
#include <queue>



#include <thirdparty/fade2d/Fade_2D.h>
#include <stdio.h>
#include <map>
#include <sstream>
#include <iomanip>

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>


#include "arena_dynamic_channel/someTools.h"
#include "arena_dynamic_channel/obstacle_collector.h"
#include "arena_mapping/mapping.h"

using namespace GEOM_FADE2D;
using namespace std;

#define inf 1 >> 30
struct DualNode;
typedef DualNode *DualNodePtr;

/* DualNode definition ------------------------------------- */
struct DualNode
{   
    // state
	enum enum_state
	{
		OPENSET = 1,
		CLOSEDSET = 2,
		UNDEFINED = 3
	};
    
	enum enum_state state{UNDEFINED};
    
    // pos
    Eigen::Vector2d pos;
    Eigen::Vector2d vel; // input vel

    // time
    double time;
    double duration;

    // index
	Eigen::Vector2i index;
    int time_index;

    // score
	double g_score, f_score;

    // parent
	DualNodePtr parent;

    DualNode(){  
		state = enum_state::UNDEFINED;
		g_score = inf;
		f_score = inf;
		parent = NULL;
    }

    ~DualNode(){};
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

/* Node comparator */
class DualNodeComparator {
public:
  bool operator()(DualNodePtr node1, DualNodePtr node2) {
    return node1->f_score > node2->f_score;
  }
};


/* define Hashtable */
template <typename T>
struct matrix_hash_dual : std::unary_function<T, size_t> {
  std::size_t operator()(T const& matrix) const {
    size_t seed = 0;
    for (size_t i = 0; i < (size_t)matrix.size(); ++i) {
      auto elem = *(matrix.data() + i);
      seed ^= std::hash<typename T::Scalar>()(elem) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    }
    return seed;
  }
};

class DualNodeHashTable {
private:
  /* data: Eigen::Vector2i index, int time_index */ 
  std::unordered_map<Eigen::Vector3i, DualNodePtr, matrix_hash_dual<Eigen::Vector3i>> data_3d_;

public:
  DualNodeHashTable(/* args */) {
  }
  ~DualNodeHashTable() {
  }

  void insert(Eigen::Vector2i idx, int time_idx, DualNodePtr node) {
    data_3d_.insert(std::make_pair(Eigen::Vector3i(idx(0), idx(1), time_idx), node));
  }

  DualNodePtr find(Eigen::Vector2i idx, int time_idx) {
    auto iter = data_3d_.find(Eigen::Vector3i(idx(0), idx(1), time_idx));
    return iter == data_3d_.end() ? NULL : iter->second;
  }

  void clear() {
    data_3d_.clear();
  }
};

/* Obstacle state ------------------------------------- */
struct ObstacleState{
    int custom_id;
    Eigen::Vector2d pos;    // current pos
    Eigen::Vector2d vel;
    
    Eigen::Vector2d pos_t;  // future pos at time t
    Point2 point2_pos_t;
    double t;

    ObstacleState(Eigen::Vector2d obs_position, Eigen::Vector2d obs_velocity, double time,int custom_index=-1){
        custom_id=custom_index;
        pos=obs_position;
        vel=obs_velocity;
        t=time;         //(obs_position-robot_position).norm()/robot_avg_velocity;
        pos_t=pos+vel*t;
        point2_pos_t=Point2(pos_t(0),pos_t(1));
    }

    ~ObstacleState(){}

    typedef std::shared_ptr<ObstacleState> Ptr;
};

/* DualGraphSearch ------------------------------------- */
class DualGraph{
private:
    /* ---------- main data structure ---------- */
    std::vector<DualNodePtr> dual_node_pool_;
    DualNodeHashTable expanded_nodes_;
    std::priority_queue<DualNodePtr, std::vector<DualNodePtr>, DualNodeComparator> open_set_;
    std::vector<DualNodePtr> dual_nodes_;

    int use_node_num_, iter_num_;

    int allocate_num_, check_num_;

    double tie_breaker_;

    bool has_path_ = false;

    /* ---------- main DT structure ---------- */
    // Delaunay Triangle Fade2D
    std::unique_ptr<Fade_2D> dt_;

    // map
    GridMap::Ptr grid_map_;
    Eigen::Vector2d occ_map_origin_,occ_map_size_2d_;
    double resolution_, inv_resolution_;
    double time_resolution_, inv_time_resolution_;
    double time_origin_;


    //obstacle
    ros::NodeHandle node_;
    std::vector<ObstacleNode::Ptr> obs_provider_nodes_;
    std::vector<ObstacleState::Ptr> obs_state_set_;
    
    // robot state variables
    bool have_odom_;
    Eigen::Vector2d odom_pos_, odom_vel_;                               // odometry state
    Eigen::Quaterniond odom_orient_;                                    // orient
    
    //robot params
    double robot_avg_vel_;                                              // robot avg velocity
    double robot_max_vel_;                                              // robot max velocity
    double radius_robot_;
    double radius_obs_;
    // plan variables
    Eigen::Vector2d start_pt_,start_vel_;
    Eigen::Vector2d end_pt_;

    // subscriber
    ros::Subscriber goal_sub_, odom_sub_;

    // publisher
    ros::Publisher vis_triangle_pub_, vis_goal_pub_;

    /* timer */
    ros::Timer update_timer_; 
    ros::Timer vis_timer_; 

    void odomCallback(const nav_msgs::OdometryConstPtr& msg);

    void goalCallback(const geometry_msgs::PoseStampedPtr& msg);
    
    // time computation
    double computeArrivingTime(Eigen::Vector2d curr_pos,Eigen::Vector2d curr_vel,Eigen::Vector2d next_pos,Eigen::Vector2d next_vel);

    std::pair<double, double> computeCollisionTime(Eigen::Vector2d pos1,Eigen::Vector2d vel1, Eigen::Vector2d pos2, Eigen::Vector2d vel2,double dist_thresh);

    // safety check
    bool isDynamicSafe(DualNodePtr curr_node, Eigen::Vector2d next_pos, double & min_to_collide_time);

    bool isGateFeasible(ObstacleState::Ptr obs1, ObstacleState::Ptr obs2, double t_arrive);

    
    double getHeuristic(Eigen::Vector2d curr_pos,Eigen::Vector2d end_pos);

    /* helper functions */
    inline void boundPosition(Eigen::Vector2d& pos);

    inline Eigen::Vector2i posToIndex(Eigen::Vector2d pt);
    
    inline int timeToIndex(double time);
    
    void retrievePath(DualNodePtr end_node);

    /* performance check */
    double dc_time_, max_dc_time_;
    int dc_update_num_;
public:
    DualGraph(){}
    ~DualGraph();
    typedef std::shared_ptr<DualGraph> Ptr;

    void init(ros::NodeHandle & nh);

    /* reset graph */
    void resetGraph();

    void searchChannel();

    void UpdateCallback(const ros::TimerEvent&);

    /* Search */
    enum { REACH_HORIZON = 1, REACH_END = 2, NO_PATH = 3, NEAR_END = 4 };
    std::vector<DualNodePtr> getVisitedNodes();

    /* visualization */
    void visualizePoints(const vector<Eigen::Vector2d>& point_set, double pt_size, const Eigen::Vector4d& color, const ros::Publisher & pub);

    void visualizeLines(const std::vector<std::pair<Eigen::Vector2d,Eigen::Vector2d>> & ptr_pair_sets, double pt_size, const Eigen::Vector4d& color, const ros::Publisher & pub);

    void publishVisGraph();

    /* reset graph for each time */
    void resetCDT(const std::vector<Eigen::Vector2d> &pt_set , const double thresh);

    void addVertice(const Eigen::Vector2d &pt);

    void removeVertice(const Eigen::Vector2d &pt);

    void setConstraintEdge(const Eigen::Vector2d &pt1, const Eigen::Vector2d &pt2);

    void getStartGoal(const Eigen::Vector2d &start_pt, const Eigen::Vector2d &end_pt);

    void visualize_vertices(Visualizer2 * vis);

    
};

inline void DualGraph::boundPosition(Eigen::Vector2d& pos) {

  pos(0) = std::max(std::min(pos(0), occ_map_size_2d_(0)), occ_map_origin_(0));
  pos(1) = std::max(std::min(pos(1), occ_map_size_2d_(1)), occ_map_origin_(1));

}

inline Eigen::Vector2i DualGraph::posToIndex(Eigen::Vector2d pos)
{
  Eigen::Vector2i idx = ((pos - occ_map_origin_) * inv_resolution_).array().floor().cast<int>();
  return idx;
}

inline int DualGraph::timeToIndex(double time)
{
  int time_idx = floor((time - time_origin_) * inv_time_resolution_);
  return time_idx;
}















#endif