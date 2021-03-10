#ifndef _DATA_HPP_
#define _DATA_HPP_

#include "graph/accessor.hpp"
#include "graph/delaunator.hpp"



namespace astar
{

/* delaunator types */
typedef delaunator::rank Rank;
typedef delaunator::index Index;
typedef delaunator::Delaunator Graph;
typedef std::shared_ptr<Graph> GraphPtr;


/* header of the pedestrian data */
constexpr size_t INIT_INDEX = 0;
constexpr size_t GOAL_INDEX = 1;
constexpr size_t PHASE1_INDEX = 2;
constexpr size_t PHASE2_INDEX = 5;
constexpr size_t PHASE3_INDEX = 4;
constexpr size_t PHASE4_INDEX = 3;
constexpr size_t PEDS_START = 6;


/* hyperpara loaded from launch file */
double SAFE_DIST = 1.0;
double MAX_SPEED = 1.8;
double TIME_HORIZON = 10.0; // time budget for
double DELTA_T = 0.2;     // time interval between graphs

double FORD_HORIZON = 1.0;    // max time used for ICS test
size_t ACTION_NUM = 72;

double CELL_SIZE = 0.1; // cell size of sdf map
double SDF_INTVL = 0.4; // time interval between sdfs
size_t SDF_NUM = 15;



/* build in hyperpara */
// const double GOAL_RADIUS = MAX_SPEED * DELTA_T * 2;
const double GOAL_RADIUS = 0.2;
const size_t OPT_STEPS = static_cast<size_t>(round(FORD_HORIZON / DELTA_T));
const size_t MIN_STEPS = static_cast<size_t>(fmin(OPT_STEPS, 3)); // at least forward 2 steps
const size_t SLICE_NUM = static_cast<size_t>(round(TIME_HORIZON / DELTA_T));
const Index NONE_INDEX = std::numeric_limits<Index>::max();



inline int min(int a, int b){
    return a<b? a:b;
}
inline size_t min(size_t a, size_t b){
    return a<b? a:b;
}


struct Vec2i
{
    Index x, y;
    Vec2i(): x(0), y(0) {}
    Vec2i(const Index xi, const Index yi): x(xi), y(yi) {}
    Vec2i(const Vec2i& vec): x(vec.x), y(vec.y) {}
    friend bool operator == (const Vec2i &a, const Vec2i &b);
};
bool operator == (const Vec2i& a, const Vec2i& b){
    return (a.x == b.x && a.y == b.y);
}


struct Vec2d
{
    double x, y;
    Vec2d():x(0.0), y(0.0){}
    Vec2d(double xp, double yp): x(xp), y(yp){}
    Vec2d(const Vec2d& vec): x(vec.x), y(vec.y) {}

    friend Vec2d operator + (const Vec2d &a, const Vec2d &b);
    friend Vec2d operator - (const Vec2d &a, const Vec2d &b);
    friend Vec2d operator * (const Vec2d &a, const Vec2d &b);
    friend Vec2d operator * (const Vec2d &a, const double scale);
    friend Vec2d operator / (const Vec2d &a, const double scale);    
    friend bool operator == (const Vec2d &a, const Vec2d &b);
    friend bool operator < (const Vec2d &a, const Vec2d &b);
    friend bool operator > (const Vec2d &a, const Vec2d &b);
    friend bool operator <= (const Vec2d &a, const Vec2d &b);
    friend bool operator >= (const Vec2d &a, const Vec2d &b);
    friend double dot (const Vec2d &a, const Vec2d &b);

    double sum() const {return x+y;}
    double angle() const { return acos(y / x);}
    double length() const { return sqrt(x*x + y*y);}
};

Vec2d operator + (const Vec2d &a, const Vec2d &b) {
    return Vec2d(a.x + b.x, a.y + b.y);
}
Vec2d operator - (const Vec2d &a, const Vec2d &b) {
    return Vec2d(a.x - b.x, a.y - b.y);
}
Vec2d operator * (const Vec2d &a, const Vec2d &b) {
    return Vec2d(a.x * b.x, a.y * b.y);
}
Vec2d operator * (const Vec2d &a, const double scale) {
    return Vec2d(a.x * scale, a.y * scale);
}
Vec2d operator / (const Vec2d &a, const double scale) {
    return Vec2d(a.x / scale, a.y / scale);
}

double dot (const Vec2d &a, const Vec2d &b) {
    return a.x * b.x + a.y * b.y;
}

bool operator == (const Vec2d &a, const Vec2d &b) {
    return (fabs(a.x - b.x) < DBL_EPSILON  && fabs(a.y - b.y) < DBL_EPSILON);
}
bool operator < (const Vec2d &a, const Vec2d &b) {
    return (a.x < b.x) && (a.y < b.y);
}
bool operator <= (const Vec2d &a, const Vec2d &b) {
    return (a.x <= b.x) && (a.y <= b.y);
}
bool operator > (const Vec2d &a, const Vec2d &b) {
    return (a.x > b.x) && (a.y > b.y);
}
bool operator >= (const Vec2d &a, const Vec2d &b) {
    return (a.x >= b.x) && (a.y >= b.y);
}


struct Edge
{
    Index a_eid, b_eid;  // the eid of two endpoints in grpah.triangles
    Vec2d a_position, b_position;  // the position of the two points
    Vec2d a_velocity, b_velocity;  // the velocity of the two points
    Vec2d a_radius, b_radius;  // used for funnel construction
    Edge(): a_eid(NONE_INDEX), b_eid(NONE_INDEX){}
    Edge(Index a, Index b): a_eid(a), b_eid(b){}
};

struct Node
{
    Index eid;  // the halfedge index = next
    Index tid;  // the triangle index = floor(eid / 3.0)
    Index sid;  // the slice index of timed graph

    Vec2d position;  // node placement position
    Vec2d velocity;  // node placement velocity

    double G, H;  // accumulated cost-to-go and heuristic
    double time_elapsed; // accumulated time
    std::shared_ptr<Node> parent;  // the parent node pointer
    std::shared_ptr<Edge> shared;  // the shared edge with parent


    Node(Index id):
        eid(id), sid(0), G(DBL_MAX), H(0.0), time_elapsed(0.0),
        parent(nullptr), shared(nullptr) {
        this->tid = static_cast<Index>(floor(eid / 3.0));
    }

    Node(Index id, Index t):
        eid(id), sid(t), G(DBL_MAX), H(0.0), time_elapsed(0.0),
        parent(nullptr), shared(nullptr) {
        this->tid = static_cast<Index>(floor(eid / 3.0));
    }

    Node(Index id, double g, double h):
        eid(id), sid(0), G(g), H(h), time_elapsed(0.0),
        parent(nullptr), shared(nullptr) {
        tid = static_cast<Index>(floor(eid / 3.0));
    }

    Node(Index id, Index t, double g, double h):
        eid(id), sid(t), G(g), H(h), time_elapsed(0.0),
        parent(nullptr), shared(nullptr) {
        tid = static_cast<Index>(floor(eid / 3.0));
    }

    void fetch_states(const Graph* graph){
        auto i = 2 * graph->triangles[eid];
        auto px = graph->coords[i];
        auto py = graph->coords[i+1];
        auto vx = graph->speeds[i];
        auto vy = graph->speeds[i+1];
        position.x = px;
        position.y = py;
        velocity.x = vx;
        velocity.y = vy;
    }

};

typedef std::shared_ptr<Edge> EdgePtr;
typedef std::shared_ptr<Node> NodePtr;


}

#endif
