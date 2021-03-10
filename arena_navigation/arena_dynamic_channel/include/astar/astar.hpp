/*
    Copyright (c) 2020, Delong Zhu <zhudelong@link.cuhk.edu.hk>
*/
#ifndef _ASTAR_HPP_
#define _ASTAR_HPP_

#include <vector>
#include <functional>
#include <set>
#include <cmath>
#include <queue>
#include <iostream>
#include <memory>
#include <algorithm>

#include "graph/delaunator.hpp"
#include "graph/accessor.hpp"
#include "data.hpp"

//#define DEBUG

namespace astar
{

/*
 * at^2 + bt + c
 *
**/
size_t solveQuadraticFunction(const double a,
                              const double b,
                              const double c,
                              double* x1,
                              double* x2){

    // TODO: check a == 0
    if (fabs(a) < DBL_EPSILON){
        *x1 = *x2 = - c / (2*b);
    }
    double D = b * b - 4 * a * c;
    if (D > 0){
        *x1=(-b+sqrt(D)) / (2*a);
        *x2=(-b-sqrt(D)) / (2*a);

        if(*x1 > *x2) {
            auto temp = *x1;
            *x1 = *x2;
            *x2 = temp;
        }
        return 2;
    } else if (fabs(D) <= DBL_EPSILON) {
        *x1 = *x2 = -b/(2*a);
        return 1;
    } else {
        return 0;
    }
}


/*
 * Perform discrete optimization to figure out the best placement
 *
 * The cost defined in this function is used to optimize placement
 * ranther than the astar search, make a clear understanding about this
 *
 * Hard Constraints
 *
**/
bool placeOptNode(const Graph* graph,
                  const Vec2d& bound_min,
                  const Vec2d& bound_max,
                  const size_t test_steps,
                  const Vec2d& robot,
                  const Vec2d& start,
                  const Vec2d& goal,
                  const std::vector<Vec2d>& actions,
                  const std::vector<Index>& peds,
                  Vec2d& opt_node,
                  size_t& opt_sid) {

    // input check
    if (bound_min >= bound_max) {
        throw std::runtime_error("placeOptNode: illegal boundaries!");
    }
    if (test_steps < 2) {
        throw std::runtime_error("placeOptNode: invalid time steps!");
    }
    if (actions.size() < 1) {
        throw std::runtime_error("placeOptNode: action set is none!");
    }


    // check whether the robot reaches goal
    auto robot_diff = goal - robot;
    auto start_diff = goal - start;
    if (robot_diff.length() < SAFE_DIST) {
        opt_node = goal;
        opt_sid = 0;
        return true;
    }

    // calculate direction
    auto robot_dir = robot_diff / robot_diff.length();
    auto start_dir = start_diff / start_diff.length();


    // start evaluation
    auto& p_r = robot;
    double max_award = -DBL_MAX;
    for (size_t k = 0; k < actions.size(); ++k) {
        // velocity contraint
        auto v_r = actions[k] * MAX_SPEED;

        // propagate time steps
        size_t ford_steps = test_steps;
        for (size_t n = 0; n < test_steps; ++n) {
            // test bound constraints
            auto p_rt = p_r + v_r * n * DELTA_T;
            if (p_rt < bound_min || p_rt > bound_max) {
                ford_steps = min(n, ford_steps);
                break;
            }
            // test collision constraints
            bool test_failed = false;
            for (size_t i = 0; i < peds.size(); ++i) {
                auto id = graph->triangles[peds[i]];
                auto v_i = Vec2d(graph->speeds[2*id], graph->speeds[2*id+1]);
                auto p_i = Vec2d(graph->coords[2*id], graph->coords[2*id+1]);

                auto p_ir = p_i - p_r;
                auto v_ir = v_i - v_r;
                auto dist = (p_ir + v_ir * n * DELTA_T).length();
                if (id >= PEDS_START && dist < SAFE_DIST) {
                    test_failed = true;
                    break;
                }
            }
            if (test_failed) {
                ford_steps = n;
                break;
            } // else - continue to propagate time
        } // end time propagation

        // the robot itset is in collision state
        if (ford_steps == 0) {
            return false;
        }
        // the robot cannot move to the next state
        if (ford_steps == 1) {
            continue;
        }
        // safe forward steps (at least one)
        ford_steps -= 1;

        // start-goal direction constraint
        if ((actions[k] * start_dir).sum() < 0) {
            return false; // backward is not allowed
        }

        // encourage moving towards the goal
        auto direction_award = ((actions[k] * robot_dir).sum() + 1.0) / 2.0;
        // encourage moving as faraway as possible
        auto distance_award = ford_steps / test_steps;
        // weighted total award
        auto total_award = 0.3 * distance_award + 0.7 * direction_award;
        // record the optimial data
        if (total_award > max_award) {
            max_award = total_award;
            opt_sid = ford_steps;
            opt_node = p_r + v_r * ford_steps * DELTA_T;
        }

    } // finish action optimization

    if (max_award < 0) {
        return false;
    } else {
        return true;
    }

}


/*
 * Define the cost used for astar searching
 * Soft constraints
 *
**/
double getOneStepCost(const NodePtr parent,
                      const NodePtr child,
                      const EdgePtr shared_edge)
{
    // time evolvement of the gate edge
    auto& p_b = shared_edge->b_position;
    auto& p_a = shared_edge->a_position;

    // normal vector of the gate edge
    auto p_ab = p_b - p_a;
    auto h_ab = p_ab.length();
    auto h_0 = 2 * SAFE_DIST;

    // cost weight
    auto hp = h_ab > h_0? 1: 1 + (h_0 - h_ab) / h_0; // [1-2]
    // std::cout << hp << std::endl;

    // gate weighted one-step cost
    return (child->position - parent->position).length() * hp;
}


/*
 *  discretize action space
 *  size: the number of discretized actions
 *
 *
**/
void genActionSet(const size_t size,
                  std::vector<Vec2d>& action_set) {
    double step = 2 * M_PI / size;
    for (size_t i = 0; i < size; ++i) {
        action_set.push_back(Vec2d(cos(step*i), sin(step*i)));
    }
}

/*
 * index actions that pass through gate <a, b> starting from p
 * a or b should not be the same with p
 *
**/
Vec2i subActionSet(const Vec2d& p,
                   const Vec2d& a,
                   const Vec2d& b,
                   const size_t size) {


    // test relative position
    auto c = (a + b) * 0.5;
    Vec2d l(a), r(b);
    if (!delaunator::isOnLeft(a.x, a.y, p.x, p.y, c.x, c.y)) {
        l = b;
        r = a;
    }

    auto pl = l - p;
    auto pr = r - p;
    double step = 2 * M_PI / size;

    double pl_rad = acos(pl.x / pl.length());
    pl_rad = pl.y < 0? 2*M_PI-pl_rad: pl_rad;

    double pr_rad = acos(pr.x / pr.length());
    pr_rad = pr.y < 0? 2*M_PI-pr_rad: pr_rad;

    auto pl_i = static_cast<size_t>(ceil(pl_rad / step));
    auto pr_i = static_cast<size_t>(floor(pr_rad / step));
    //std::cout<<"---------------"<<std::endl;
    //std::cout<<"pr_rad"<<pr_rad<<std::endl;
    //std::cout<<"pl_rad"<<pl_rad<<std::endl;

    //std::cout<<"pl x:"<<pl.x<<"  pl y:"<<pl.y<<std::endl;
    //std::cout<<"pr x:"<<pr.x<<"  pr y:"<<pr.y<<std::endl;

    
    //std::cout<<"---------------"<<std::endl;

    return Vec2i(pl_i, pr_i);

}



/*
 * discretize the time space and action space
 * the state space remains unchanged
 *
**/
void discretizeSpaces(const std::vector<double>& coords,
                      const std::vector<double>& speeds,
                      const std::vector<double>& angles,
                      std::vector<GraphPtr>& timed_graph,
                      std::vector<Vec2d>& action_set) {

    // clear data
    action_set.clear();
    timed_graph.clear();

    // discretize the action sapce
    genActionSet(ACTION_NUM, action_set);

    // decode the boundaries
    auto x_min = coords[PHASE3_INDEX*2];
    auto y_min = coords[PHASE3_INDEX*2+1];
    auto x_max = coords[PHASE1_INDEX*2];
    auto y_max = coords[PHASE1_INDEX*2+1];

    // decode the init and goal
    auto x_init = coords[INIT_INDEX*2];
    auto y_init = coords[INIT_INDEX*2+1];
    auto x_goal = coords[GOAL_INDEX*2];
    auto y_goal = coords[GOAL_INDEX*2+1];

    // discretize the time space
    for (size_t t = 0; t < SLICE_NUM; ++t) {
        std::vector<double> coord_t, speed_t, angle_t;

        // push init and goal
        coord_t.push_back(x_init);
        coord_t.push_back(y_init);
        coord_t.push_back(x_goal);
        coord_t.push_back(y_goal);
        for (size_t i = 0; i < 4; ++i) {
            speed_t.push_back(0.0);
            angle_t.push_back(0.0);
        }

        // time evolution
        // std::cout << "new node:" << std::endl;
        for (size_t i = 4; i < coords.size(); i += 2) {
            auto xi = coords[i] + speeds[i] * DELTA_T * t;
            auto yi = coords[i+1] + speeds[i+1] * DELTA_T * t;
            if (xi < x_min || xi > x_max || yi < y_min || yi > y_max){
                continue;
            }
            // std::cout << xi << " " << yi << std::endl;
            // update state space
            coord_t.push_back(xi);
            coord_t.push_back(yi);
            speed_t.push_back(speeds[i]);
            speed_t.push_back(speeds[i+1]);
            angle_t.push_back(angles[i]);
            angle_t.push_back(angles[i+1]);
        }

        // generate timed graph and timed close list
        auto graph_t = std::make_shared<Graph>(coord_t, speed_t, angle_t);
        timed_graph.push_back(graph_t);
    }

}

void printVertex(const Graph* graph, const Index eid) {
    using namespace std;
    using namespace delaunator;
    auto ai = 2 * graph->triangles[eid];
    cout << eid << ": "<< graph->coords[ai] << "," << graph->coords[ai+1] << endl;
    
    ai = 2 * graph->triangles[nextHalfedge(eid)];

    cout << nextHalfedge(eid) << ": "<< graph->coords[ai] << "," << graph->coords[ai+1] << endl;
    
    ai = 2 * graph->triangles[prevHalfedge(eid)];
    cout << prevHalfedge(eid) << ": " << graph->coords[ai] << "," << graph->coords[ai+1] << endl;
}



/*
 * Assume the first two are init and goal
 *
 * 1. after delta_t, the ancor should lie in the next graph
 * 2. when searching neighbor, we need to do that in the next graph
 * 3. randomrize a tiny shift of the init to perfect the face localization
 *
 * init: the current position of the robot, which is different from the
 * start position at the beginning.
 *
**/
bool StateTimeAstar(const std::vector<double>& coords,
                    const std::vector<double>& speeds,
                    const std::vector<double>& angles,
                    const Vec2d& robot,
                    const Vec2d& goal,
                    std::vector<Vec2d>& waypoints){

    namespace dl = delaunator;
    waypoints.clear();
    // already reach the goal
    if ((goal - robot).length() < GOAL_RADIUS) {
        std::cout << "StateTimeAstar: goal reached!" << std::endl;
        waypoints.push_back(goal);
        waypoints.push_back(robot);
        return true;
    }

    // discretize time sapce and action space
    std::vector<Vec2d> action_set;
    std::vector<GraphPtr> timed_graph;
    discretizeSpaces(coords, speeds, angles, timed_graph, action_set);

    // decode boundaries and init of state space
    auto x_min = coords[PHASE3_INDEX*2];
    auto y_min = coords[PHASE3_INDEX*2+1];
    auto x_max = coords[PHASE1_INDEX*2];
    auto y_max = coords[PHASE1_INDEX*2+1];
    auto x_init = coords[INIT_INDEX*2];
    auto y_init = coords[INIT_INDEX*2+1];
    Vec2d bbx_init(x_init, y_init);
    Vec2d bbx_min(x_min, y_min), bbx_max(x_max, y_max);

    

    // initialize open list
    auto cmp = [](const NodePtr& a, const NodePtr& b) {return (a->G + a->H) > (b->G + b->H);};
    std::priority_queue<NodePtr, std::vector<NodePtr>, decltype(cmp)> open_list(cmp);
  
    // initialize closed list
    typedef std::vector<NodePtr> NodePtrVec;
    std::vector<NodePtrVec> closed_list;
    for (size_t t = 0; t < SLICE_NUM; ++t) {
        // triangles size = <v1, v2, v3> * triangles_number
        size_t tri_size = static_cast<size_t>(floor(timed_graph[t]->triangles.size() / 3.0));
        closed_list.push_back(NodePtrVec(tri_size, nullptr));
    }

  
    // locate robot in start graph
    auto init_eid = dl::locateCurrentFace(timed_graph[0].get(), robot.x, robot.y);
    if (init_eid == NONE_INDEX) {
        std::cout << "Invalid start position!" << std::endl;
        return false;
    }

    /* important - alreay reach the goal triangle */
    if (floor(init_eid / 3.0) == GOAL_INDEX) {
        waypoints.push_back(goal);
        waypoints.push_back(robot);
        std::cout<<"[time astar search]: alreay reach the goal triangle---------------"<<std::endl;
        return true;
    }

    
    // seed the start node
    auto h_value = (goal - robot).length();
    double g_value = 0.0;
    size_t init_slice = 0;
    auto start_ptr = std::make_shared<Node>(init_eid, init_slice, g_value, h_value);
    start_ptr->position = robot;
    start_ptr->velocity = Vec2d(MAX_SPEED, MAX_SPEED);
    open_list.push(start_ptr);

    //std::cout<<"[time astar search]: begin while loop search---------------"<<std::endl;
    // begin astar searching
    double nearest_dist = DBL_MAX;
    NodePtr goal_reached(nullptr);
    NodePtr goal_nearest(nullptr);
    while (!open_list.empty()) {
        
        NodePtr parent = open_list.top();
        open_list.pop();
       
        // the goal or time horizon is reached
        double dist_to_goal = (parent->position - goal).length();
        if (dist_to_goal < GOAL_RADIUS || parent->sid == SLICE_NUM-1) {
            goal_reached = parent;
            break;
        }
        
        // record the nearest node
        if (dist_to_goal < nearest_dist) {
            nearest_dist = dist_to_goal;
            goal_nearest = parent;
        }
        
        // already in the closed list
        if (closed_list[parent->sid][parent->tid] != nullptr) {
            continue;
        } else {
            closed_list[parent->sid][parent->tid] = parent;
        }

        
        // retrive the target slice
        auto graph = timed_graph[parent->sid].get();
        std::vector<Index> neighbors; // eid of neighbor edges
        std::vector<Index> outgoings; // eid of self edges
        std::vector<Index> pedestrians; // eid of nearby pedestrians
        
        // retrieve neighbor triangles in the target slice
        dl::neighborVertices(graph, parent->eid, pedestrians);
        dl::neighborTriangles(graph, parent->eid, neighbors, outgoings);
        

#ifdef DEBUG
        std::cout << "parent eid: " << parent->eid << std::endl;
        std::cout << "parent pos: "
                  << parent->position.x << ", " << parent->position.y << std::endl;
#endif
        int same_counter=1;
        for (size_t i = 0; i < neighbors.size(); ++i) {
            
            // child and
            auto c_eid = neighbors[i];
            auto p_eid = outgoings[i];
            
            auto c_vertex = std::make_shared<Node>(c_eid);
            auto p_vertex = std::make_shared<Node>(p_eid);
            
            c_vertex->fetch_states(graph);
            p_vertex->fetch_states(graph);

            if(parent->position==c_vertex->position){
                //std::cout<<"same:"<<same_counter<<"***********************************"<<std::endl;
                //std::cout<<"parent->eid:"<<parent->eid<<std::endl;
                //std::cout<<"c_vertex->eid:"<<c_vertex->eid<<std::endl;
                
                same_counter++;
                continue;
            }

            if(parent->position==p_vertex->position){
                //std::cout<<"same:"<<same_counter<<"***********************************"<<std::endl;
                //std::cout<<"parent->eid:"<<parent->eid<<std::endl;
                //std::cout<<"p_vertex->eid:"<<p_vertex->eid<<std::endl;
                same_counter++;
                continue;
            }
            
#ifdef DEBUG
            std::cout << i << "th neighbor eid: " << c_eid << std::endl;
            printVertex(graph, c_eid);
#endif
            // subset of actions that toward the child triangle
            std::vector<Vec2d> sub_actions;

            auto sub_idx = subActionSet(parent->position, c_vertex->position,
                                        p_vertex->position, ACTION_NUM);
            
            //std::cout<<"time astar search704---------------"<<std::endl;
            //std::cout<<"sub_idx.x="<<sub_idx.x<<std::endl;
            //std::cout<<"sub_idx.y="<<sub_idx.y<<std::endl;
            if (sub_idx.x < sub_idx.y) {
                sub_actions.assign(action_set.begin() + sub_idx.x, action_set.begin() + sub_idx.y + 1);
                                   
            } else { // cross zero point
                sub_actions.assign(action_set.begin() + sub_idx.x, action_set.end());
                sub_actions.assign(action_set.begin(), action_set.begin() + sub_idx.y + 1);
            }

            
            // calculate the optimal node placement
            Vec2d opt_node;
            size_t opt_sid;
            size_t test_steps = min(OPT_STEPS, SLICE_NUM-parent->sid);
            if (!placeOptNode(graph, bbx_min, bbx_max, test_steps, parent->position,
                              bbx_init, goal, sub_actions, pedestrians, opt_node, opt_sid)) {
                // the parent is in danger itself or trapped in the parent position
                // note that the collision is checked in the time-evoluted parent graph
                continue;
            }
        
            // locate the node in target slice
            size_t child_sid = parent->sid + opt_sid;
            Index child_eid = dl::locateCurrentFace(timed_graph[child_sid].get(), opt_node.x, opt_node.y);
            if (child_eid == NONE_INDEX) {
                continue;
            }
            auto child = std::make_shared<Node>(child_eid, child_sid);
            child->position = opt_node;
            child->velocity = Vec2d(MAX_SPEED, MAX_SPEED);

            
            // generate pedestrian gate for navi cost calculation
            auto gate = std::make_shared<Edge>(p_eid, c_eid);
            gate->a_position = p_vertex->position;
            gate->a_velocity = p_vertex->velocity;
            gate->b_position = c_vertex->position;
            gate->b_velocity = c_vertex->velocity;
            double navi_cost = getOneStepCost(parent, child, gate);
            
            // update child and insert to open lisht
            if (parent->G + navi_cost < child->G){
                child->G = parent->G + navi_cost;
                child->H = (child->position - goal).length();
                child->parent = parent;
                child->shared = gate;
                open_list.push(child);
            }

            
        } // end processing neighbors

    } // end while loop

    //std::cout<<"[time astar search]: while loop done---------------"<<std::endl;
    // back-track the path
    if (goal_nearest == nullptr) {
        std::cout<<"[time astar search]: done[1] not found any thing---------------"<<std::endl;
        return false;
    }
    
    auto ancestor = goal_reached;
    if (ancestor == nullptr) {
        ancestor = goal_nearest;
    }
   
    // push_back goal, ancestor not lies in goal region
    if ((ancestor->position - goal).length() >= GOAL_RADIUS) {
        waypoints.push_back(goal);
    }
    // push_back ohther waypoint
    while (ancestor != nullptr) {
        waypoints.push_back(ancestor->position);
        ancestor = ancestor->parent;
    }
    std::cout<<"[time astar search]: done[2] found waypoints---------------"<<std::endl;
#ifdef DEBUG
    std::cout << "waypoints number: " << waypoints.size() << std::endl;
    for (uint i = 0; i < waypoints.size(); ++i) {
        std::cout << waypoints[i].x << ", " << waypoints[i].y << std::endl;
    }
#endif
    return true;

}




// end namespace
}
#endif // __ASTAR_HPP_8F637DB91972F6C878D41D63F7E7214F__

