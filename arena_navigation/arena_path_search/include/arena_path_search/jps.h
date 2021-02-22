
#ifndef _A_STAR_H_
#define _A_STAR_H_



#include <Eigen/Eigen>
#include <iostream>
#include <string>
#include <ros/console.h>
#include <ros/ros.h>
#include <memory>

#include "arena_path_search/graph_search.h"
#include "arena_mapping/mapping.h"

class JPS: public GraphSearch
{
private:
    // occ map
    GridMap::Ptr grid_map_;
    Eigen::Vector2d occ_map_origin_,occ_map_size_2d_;

    // map param

    // search param
    bool use_jps_;
    
    /* helper */
    bool ConvertToIndexAndAdjustStartEndPoints( Eigen::Vector2d start_pt,  Eigen::Vector2d end_pt, Eigen::Vector2i &start_idx, Eigen::Vector2i &end_idx);

    bool isOccupied(const Eigen::Vector2d &pos){ return (bool)grid_map_->getFusedInflateOccupancy(pos); }

    std::vector<GridNodePtr> retrievePath(GridNodePtr currentPtr);

    /* get Successor nodes */
    void AstarGetSucc(GridNodePtr currentPtr, std::vector<GridNodePtr> & neighborPtrSets, std::vector<double> & edgeCostSets);

    void JPSSucc(GridNodePtr currentPtr, std::vector<GridNodePtr> & neighborPtrSets, std::vector<double> & edgeCostSets);

public:
    JPS(){};
    ~JPS();
    typedef std::shared_ptr<JPS> Ptr;

    void setParam();

    void setEnvironment(const GridMap::Ptr& env);
    
    void setSearchMap(const Eigen::Vector2i pool_size, double step_size, double lamda_heu=0.1, bool use_jps=false);

    bool search( Eigen::Vector2d start_pt, Eigen::Vector2d end_pt, bool is_local=false);

    std::vector<Eigen::Vector2d> getPath();
};






















#endif