#ifndef NE_GRID_SEARCH_HPP
#define NE_GRID_SEARCH_HPP

#include <boost/heap/d_ary_heap.hpp>

#include <limits>
#include <memory>
#include <unordered_map>
#include <vector>

#include <Eigen/Dense>

#include "ne_esdf_builder/esdf_builder_node.hpp"

namespace JPS
{

using Vec2 = Eigen::Vector2d;

struct sentry_state;

template<class T>
struct compare_state
{
    bool operator()(T a1, T a2) const
    {
        const double f1 = a1->g + a1->h;
        const double f2 = a2->g + a2->h;
        if ((f1 >= f2 - 0.000001) && (f1 <= f2 + 0.000001)) {
            return a1->g < a2->g;
        }
        return f1 > f2;
    }
};

using StatePtr = std::shared_ptr<sentry_state>;
using priorityQueue = boost::heap::d_ary_heap<
    StatePtr,
    boost::heap::mutable_<true>,
    boost::heap::arity<2>,
    boost::heap::compare<compare_state<StatePtr>>>;

struct sentry_state
{
    int id;
    int x;
    int y = 0;
    int dx;
    int dy = 0;
    int parentId = -1;
    priorityQueue::handle_type heapkey;
    double g = std::numeric_limits<double>::infinity();
    double h = 0.0;
    bool opened = false;
    bool closed = false;

    sentry_state(int id, int x, int y, int dx, int dy)
    : id(id), x(x), y(y), dx(dx), dy(dy) {}
};

struct JPS2DNeib
{
    int ns[9][2][8];
    int f1[9][2][2];
    int f2[9][2][2];
    static constexpr int nsz[3][2] = {{8, 0}, {1, 2}, {3, 2}};

    void print();
    JPS2DNeib();

private:
    void Neib(int dx, int dy, int norm1, int dev, int & tx, int & ty);
    void FNeib(int dx, int dy, int norm1, int dev, int & fx, int & fy, int & nx, int & ny);
};

class EsdfSearch
{
public:
    EsdfSearch(std::shared_ptr<esdf_builder::ESDFMap2D> map_ptr, const double & safe_dis);

    bool plan(int xStart, int yStart, int xGoal, int yGoal, int maxExpand = -1);
    std::vector<StatePtr> getPath() const;
    std::vector<StatePtr> getOpenSet() const;
    std::vector<StatePtr> getCloseSet() const;
    std::vector<StatePtr> getAllSet() const;
    void SetsafeDis(const double & safe_dis);
    double get_Safedis() const;

private:
    bool plan(StatePtr & currNode_ptr, int max_expand, int start_id, int goal_id);
    void getJPSSucc(const StatePtr & curr_Node, std::vector<int> & succ_ids, std::vector<double> & succ_costs);
    std::vector<StatePtr> recoverPath(StatePtr node, int start_id);
    int coordToId(int x, int y) const;
    bool isFree(int x, int y) const;
    bool isUnoccupied(int x, int y) const;
    bool isOccupied(int x, int y) const;
    double getHeur(int x, int y) const;
    bool hasForced(int x, int y, int dx, int dy) const;
    bool jump(int x, int y, int dx, int dy, int & new_x, int & new_y);

    std::shared_ptr<JPS2DNeib> jps_2d;
    std::vector<std::vector<int>> ns_;
    std::shared_ptr<esdf_builder::ESDFMap2D> map_;
    std::vector<StatePtr> path_;
    double xGoal_{0.0};
    double yGoal_{0.0};
    priorityQueue pq_;
    double safe_dis_{0.0};
    std::vector<bool> seen_;
    std::unordered_map<int, StatePtr> hm_;
    int xDim_{0};
    int yDim_{0};
    bool verbose_{false};
};

}  // namespace JPS

#endif
