
#ifndef _DUAL_GRAPH_SEARCH_H
#define _DUAL_GRAPH_SEARCH_H

#include <Eigen/Eigen>
#include <iostream>
#include <string>
#include <ros/console.h>
#include <ros/ros.h>

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

#include "arena_dynamic_channel/someTools.h"
using namespace GEOM_FADE2D;
using namespace std;

#define inf 1 >> 30
struct DualNode;
typedef DualNode *DualNodePtr;

// struct DualNode
// {   
//     // state
// 	enum enum_state
// 	{
// 		OPENSET = 1,
// 		CLOSEDSET = 2,
// 		UNDEFINED = 3
// 	};
// 	enum enum_state state
// 	{UNDEFINED};
    
//     // index
// 	Eigen::Vector2i index;

//     // score
// 	double gScore{inf}, fScore{inf};

//     // cameFrom
// 	DualNodePtr cameFrom{NULL};

//     // direction of expanding( for jps only )
//     Eigen::Vector2i dir; 

//     int rounds{0}; // Distinguish every call
    
//     DualNodePtr(Eigen::Vector2i _index){  
// 		state = enum_state::UNDEFINED;
// 		index = _index;
// 		dir   = Eigen::Vector2i::Zero();
// 		gScore = inf;
// 		fScore = inf;
// 		cameFrom = NULL;
//     }

//     DualNodePtr(){};
//     ~DualNodePtr(){};
// };

// /* Node comparator */
// class DualNodeComparator {
// public:
//   bool operator()(DualNodePtr node1, DualNodePtr node2) {
//     return node1->f_score > node2->f_score;
//   }
// };


// /* define Hashtable */
// template <typename T>
// struct matrix_hash0 : std::unary_function<T, size_t> {
//   std::size_t operator()(T const& matrix) const {
//     size_t seed = 0;
//     for (size_t i = 0; i < (size_t)matrix.size(); ++i) {
//       auto elem = *(matrix.data() + i);
//       seed ^= std::hash<typename T::Scalar>()(elem) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
//     }
//     return seed;
//   }
// };


// class DualNodeHashTable {
// private:
//   /* data */
//   std::unordered_map<Eigen::Vector2i, DualNodePtr, matrix_hash0<Eigen::Vector2i>> data_2d_;
//   std::unordered_map<Eigen::Vector3i, DualNodePtr, matrix_hash0<Eigen::Vector3i>> data_3d_;

// public:
//   DualNodeHashTable(/* args */) {
//   }
//   ~DualNodeHashTable() {
//   }
//   void insert(Eigen::Vector2i idx, DualNodePtr node) {
//     data_2d_.insert(std::make_pair(idx, node));
//   }
//   void insert(Eigen::Vector2i idx, int time_idx, DualNodePtr node) {
//     data_3d_.insert(std::make_pair(Eigen::Vector3i(idx(0), idx(1), time_idx), node));
//   }

//   DualNodePtr find(Eigen::Vector2i idx) {
//     auto iter = data_2d_.find(idx);
//     return iter == data_2d_.end() ? NULL : iter->second;
//   }
//   DualNodePtr find(Eigen::Vector2i idx, int time_idx) {
//     auto iter = data_3d_.find(Eigen::Vector3i(idx(0), idx(1), time_idx));
//     return iter == data_3d_.end() ? NULL : iter->second;
//   }

//   void clear() {
//     data_2d_.clear();
//     data_3d_.clear();
//   }
// };














class DualGraph{
private:
    std::unique_ptr<Fade_2D> dt_;
    std::vector<Eigen::Vector2d> input_points_set_;



    Eigen::Vector2d start_pt_;
    Eigen::Vector2d end_pt_;


public:
    DualGraph(){}
    ~DualGraph();
    typedef std::shared_ptr<DualGraph> Ptr;

    void init();

    /* reset graph for each time */
    void resetCDT(const std::vector<Eigen::Vector2d> &pt_set , const double thresh);

    void addVertice(const Eigen::Vector2d &pt);

    void removeVertice(const Eigen::Vector2d &pt);

    void setConstraintEdge(const Eigen::Vector2d &pt1, const Eigen::Vector2d &pt2);

    void getStartGoal(const Eigen::Vector2d &start_pt, const Eigen::Vector2d &end_pt);

    void visualize_vertices(Visualizer2 * vis);
};


















#endif