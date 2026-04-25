/*
 * @Author: ei_code_bash && 3080152159@qq.com
 * @Date: 2026-03-30 22:27:11
 * @LastEditors: ei_code_bash && 3080152159@qq.com
 * @LastEditTime: 2026-04-19 15:55:21
 * @FilePath: /ne_nav/src/ne_global_planner/src/ne_grid_search.cpp
 * @Description: 我永远喜欢雪之下雪乃
 *
 * Copyright (c) 2026 by ei_code_bash, All Rights Reserved.
 */
#include "ne_global_planner/ne_grid_search.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>

namespace JPS
{

namespace
{

static int norm(int dx, int dy)
{
    return std::abs(dx) + std::abs(dy);
}

}  // namespace

JPS2DNeib::JPS2DNeib()
{
    int id = 0;
    for (int dy = -1; dy <= 1; ++dy) {
        for (int dx = -1; dx <= 1; ++dx) {
            const int norm1 = std::abs(dx) + std::abs(dy);
            for (int dev = 0; dev < nsz[norm1][0]; ++dev) {
                Neib(dx, dy, norm1, dev, ns[id][0][dev], ns[id][1][dev]);
            }
            for (int dev = 0; dev < nsz[norm1][1]; ++dev) {
                FNeib(
                    dx, dy, norm1, dev,
                    f1[id][0][dev], f1[id][1][dev],
                    f2[id][0][dev], f2[id][1][dev]);
            }
            ++id;
        }
    }
}

void JPS2DNeib::Neib(int dx, int dy, int norm1, int dev, int & tx, int & ty)
{
    switch (norm1) {
        case 0:
            switch (dev) {
                case 0: tx = 1; ty = 0; return;
                case 1: tx = -1; ty = 0; return;
                case 2: tx = 0; ty = 1; return;
                case 3: tx = 1; ty = 1; return;
                case 4: tx = -1; ty = 1; return;
                case 5: tx = 0; ty = -1; return;
                case 6: tx = 1; ty = -1; return;
                case 7: tx = -1; ty = -1; return;
            }
            break;
        case 1:
            tx = dx;
            ty = dy;
            return;
        case 2:
            switch (dev) {
                case 0: tx = dx; ty = 0; return;
                case 1: tx = 0; ty = dy; return;
                case 2: tx = dx; ty = dy; return;
            }
            break;
        default:
            break;
    }
}

void JPS2DNeib::FNeib(int dx, int dy, int norm1, int dev, int & fx, int & fy, int & nx, int & ny)
{
    switch (norm1) {
        case 1:
            switch (dev) {
                case 0: fx = 0; fy = 1; break;
                case 1: fx = 0; fy = -1; break;
                default: fx = 0; fy = 0; break;
            }

            if (dx == 0) {
                fx = fy;
                fy = 0;
            }

            nx = dx + fx;
            ny = dy + fy;
            return;
        case 2:
            switch (dev) {
                case 0:
                    fx = -dx;
                    fy = 0;
                    nx = -dx;
                    ny = dy;
                    return;
                case 1:
                    fx = 0;
                    fy = -dy;
                    nx = dx;
                    ny = -dy;
                    return;
                default:
                    break;
            }
            break;
        default:
            break;
    }

    fx = 0;
    fy = 0;
    nx = 0;
    ny = 0;
}

void JPS2DNeib::print()
{
    for (int dx = -1; dx <= 1; ++dx) {
        for (int dy = -1; dy <= 1; ++dy) {
            const int id = (dx + 1) + 3 * (dy + 1);
            std::printf("[dx: %d, dy: %d]-->id: %d:\n", dx, dy, id);
            for (unsigned int i = 0; i < sizeof(f1[id][0]) / sizeof(f1[id][0][0]); ++i) {
                std::printf("                f1: [%d, %d]\n", f1[id][0][i], f1[id][1][i]);
            }
        }
    }
}

EsdfSearch::EsdfSearch(std::shared_ptr<esdf_builder::ESDFMap2D> map_ptr, const double & safe_dis)
: map_(std::move(map_ptr)), safe_dis_(safe_dis)
{
    const Eigen::Vector2i voxel_num = map_ ? map_->getVoxelNum() : Eigen::Vector2i::Zero();
    xDim_ = voxel_num.x();
    yDim_ = voxel_num.y();
    jps_2d = std::make_shared<JPS2DNeib>();

    ns_.resize(9);
    for (int id = 0; id < 9; ++id) {
        const int idx = (id % 3) - 1;
        const int idy = (id / 3) - 1;
        const int normVal = norm(idx, idy);
        for (int i = 0; i < JPS2DNeib::nsz[normVal][0]; ++i) {
            const int enc = (jps_2d->ns[id][0][i] + 1) + 3 * (jps_2d->ns[id][1][i] + 1);
            ns_[id].push_back(enc);
        }
    }
}

void EsdfSearch::SetsafeDis(const double & safe_dis)
{
    safe_dis_ = safe_dis;
}

double EsdfSearch::get_Safedis() const
{
    return safe_dis_;
}

int EsdfSearch::coordToId(int x, int y) const
{
    return x + y * xDim_;
}

bool EsdfSearch::isOccupied(int x, int y) const
{
    if (!map_ || x < 0 || x >= xDim_ || y < 0 || y >= yDim_) {
        return true;
    }
    return map_->isOccupied(Eigen::Vector2i(x, y));
}

bool EsdfSearch::isFree(int x, int y) const
{
    if (!map_ || x < 0 || x >= xDim_ || y < 0 || y >= yDim_) {
        return false;
    }

    const Eigen::Vector2i idx(x, y);
    if (map_->isOccupied(idx)) {
        return false;
    }

    Eigen::Vector2d pos;
    map_->indexToPos(idx, pos);
    return !map_->isCollision(pos, safe_dis_);
}

bool EsdfSearch::isUnoccupied(int x, int y) const
{
    return isFree(x, y);
}

double EsdfSearch::getHeur(int x, int y) const
{
    const double dx = static_cast<double>(x) - xGoal_;
    const double dy = static_cast<double>(y) - yGoal_;
    return std::sqrt(dx * dx + dy * dy);
}

bool EsdfSearch::hasForced(int x, int y, int dx, int dy) const
{
    const int id = (dx + 1) + 3 * (dy + 1);
    const int normVal = norm(dx, dy);

    for (int dev = 0; dev < JPS2DNeib::nsz[normVal][1]; ++dev) {
        const int fn_dx = jps_2d->f1[id][0][dev];
        const int fn_dy = jps_2d->f1[id][1][dev];
        const int forced_dx = jps_2d->f2[id][0][dev];
        const int forced_dy = jps_2d->f2[id][1][dev];

        if (!isFree(x + fn_dx, y + fn_dy) && isFree(x + forced_dx, y + forced_dy)) {
            return true;
        }
    }
    return false;
}

bool EsdfSearch::jump(int x, int y, int dx, int dy, int & new_x, int & new_y)
{
    new_x = x + dx;
    new_y = y + dy;

    if (!isFree(new_x, new_y)) {
        return false;
    }

    if (dx != 0 && dy != 0) {
        // Relax diagonal motion: only block it when both axial side cells are unavailable.
        // This keeps JPS from dying early in narrow corner-like passages.
        if (!isFree(x + dx, y) && !isFree(x, y + dy)) {
            return false;
        }
    }

    if (coordToId(new_x, new_y) == coordToId(static_cast<int>(xGoal_), static_cast<int>(yGoal_))) {
        return true;
    }

    if (hasForced(new_x, new_y, dx, dy)) {
        return true;
    }

    if (dx != 0 && dy != 0) {
        int tmp_x = 0;
        int tmp_y = 0;
        if (jump(new_x, new_y, dx, 0, tmp_x, tmp_y)) {
            return true;
        }
        if (jump(new_x, new_y, 0, dy, tmp_x, tmp_y)) {
            return true;
        }
    }

    return jump(new_x, new_y, dx, dy, new_x, new_y);
}

void EsdfSearch::getJPSSucc(const StatePtr & curr_Node, std::vector<int> & succ_ids, std::vector<double> & succ_costs)
{
    succ_ids.clear();
    succ_costs.clear();

    const int x = curr_Node->x;
    const int y = curr_Node->y;
    const int dx = curr_Node->dx;
    const int dy = curr_Node->dy;
    const int id = (dx + 1) + 3 * (dy + 1);
    const int normVal = norm(dx, dy);

    for (int i = 0; i < JPS2DNeib::nsz[normVal][0]; ++i) {
        const int n_dx = jps_2d->ns[id][0][i];
        const int n_dy = jps_2d->ns[id][1][i];

        int new_x = 0;
        int new_y = 0;
        if (jump(x, y, n_dx, n_dy, new_x, new_y)) {
            const int s_id = coordToId(new_x, new_y);
            const double cost = std::sqrt(
                static_cast<double>(new_x - x) * static_cast<double>(new_x - x) +
                static_cast<double>(new_y - y) * static_cast<double>(new_y - y));
            succ_ids.push_back(s_id);
            succ_costs.push_back(cost);
        }
    }

    for (int dev = 0; dev < JPS2DNeib::nsz[normVal][1]; ++dev) {
        const int fn_dx = jps_2d->f1[id][0][dev];
        const int fn_dy = jps_2d->f1[id][1][dev];

        if (isFree(x + fn_dx, y + fn_dy)) {
            continue;
        }

        const int f_dx = jps_2d->f2[id][0][dev];
        const int f_dy = jps_2d->f2[id][1][dev];

        int new_x = 0;
        int new_y = 0;
        if (jump(x, y, f_dx, f_dy, new_x, new_y)) {
            const int s_id = coordToId(new_x, new_y);
            const double cost = std::sqrt(
                static_cast<double>(new_x - x) * static_cast<double>(new_x - x) +
                static_cast<double>(new_y - y) * static_cast<double>(new_y - y));
            succ_ids.push_back(s_id);
            succ_costs.push_back(cost);
        }
    }
}

bool EsdfSearch::plan(int xStart, int yStart, int xGoal, int yGoal, int maxExpand)
{
    if (!map_) {
        return false;
    }

    const Eigen::Vector2i voxel_num = map_->getVoxelNum();
    xDim_ = voxel_num.x();
    yDim_ = voxel_num.y();

    pq_.clear();
    path_.clear();
    seen_.clear();
    seen_.resize(static_cast<std::size_t>(xDim_ * yDim_), false);

    xGoal_ = static_cast<double>(xGoal);
    yGoal_ = static_cast<double>(yGoal);

    if (!isFree(xStart, yStart)) {
        std::printf("[JPS] Start (%d, %d) is NOT free, map size: %d x %d\n", xStart, yStart, xDim_, yDim_);
        return false;
    }
    if (!isFree(xGoal, yGoal)) {
        std::printf("[JPS] Goal (%d, %d) is NOT free, map size: %d x %d\n", xGoal, yGoal, xDim_, yDim_);
        return false;
    }

    std::printf("[JPS] Planning from (%d,%d) to (%d,%d), map: %d x %d\n", xStart, yStart, xGoal, yGoal, xDim_, yDim_);

    const int start_id = coordToId(xStart, yStart);
    const int goal_id = coordToId(xGoal, yGoal);

    auto startNode = std::make_shared<sentry_state>(start_id, xStart, yStart, 0, 0);
    startNode->g = 0.0;
    startNode->h = getHeur(xStart, yStart);

    return plan(startNode, maxExpand, start_id, goal_id);
}

bool EsdfSearch::plan(StatePtr & currNode_ptr, int max_expand, int start_id, int goal_id)
{
    hm_.clear();

    currNode_ptr->heapkey = pq_.push(currNode_ptr);
    currNode_ptr->opened = true;
    seen_[static_cast<std::size_t>(currNode_ptr->id)] = true;
    hm_[currNode_ptr->id] = currNode_ptr;

    int expand_count = 0;

    while (!pq_.empty()) {
        StatePtr curr = pq_.top();
        pq_.pop();
        curr->closed = true;

        if (curr->id == goal_id) {
            path_ = recoverPath(curr, start_id);
            return true;
        }

        ++expand_count;
        if (max_expand > 0 && expand_count >= max_expand) {
            return false;
        }

        std::vector<int> succ_ids;
        std::vector<double> succ_costs;
        getJPSSucc(curr, succ_ids, succ_costs);

        for (std::size_t i = 0; i < succ_ids.size(); ++i) {
            const int succ_id = succ_ids[i];
            const double succ_cost = succ_costs[i];
            const int succ_x = succ_id % xDim_;
            const int succ_y = succ_id / xDim_;
            const int succ_dx = (succ_x == curr->x) ? 0 : (succ_x > curr->x ? 1 : -1);
            const int succ_dy = (succ_y == curr->y) ? 0 : (succ_y > curr->y ? 1 : -1);
            const double new_g = curr->g + succ_cost;

            auto it = hm_.find(succ_id);
            if (it == hm_.end()) {
                auto newNode = std::make_shared<sentry_state>(succ_id, succ_x, succ_y, succ_dx, succ_dy);
                newNode->g = new_g;
                newNode->h = getHeur(succ_x, succ_y);
                newNode->parentId = curr->id;
                newNode->opened = true;
                newNode->heapkey = pq_.push(newNode);
                hm_[succ_id] = newNode;
                seen_[static_cast<std::size_t>(succ_id)] = true;
            } else {
                StatePtr & existNode = it->second;
                if (existNode->closed) {
                    continue;
                }
                if (new_g < existNode->g) {
                    existNode->g = new_g;
                    existNode->dx = succ_dx;
                    existNode->dy = succ_dy;
                    existNode->parentId = curr->id;
                    pq_.update(existNode->heapkey);
                }
            }
        }
    }

    return false;
}

std::vector<StatePtr> EsdfSearch::recoverPath(StatePtr node, int start_id)
{
    std::vector<StatePtr> path;
    path.push_back(node);
    while (node && node->id != start_id) {
        auto it = hm_.find(node->parentId);
        if (it == hm_.end()) {
            break;
        }
        node = it->second;
        path.push_back(node);
    }
    std::reverse(path.begin(), path.end());
    return path;
}

std::vector<StatePtr> EsdfSearch::getPath() const
{
    return path_;
}

std::vector<StatePtr> EsdfSearch::getOpenSet() const
{
    std::vector<StatePtr> open;
    for (const auto & item : hm_) {
        const auto & ptr = item.second;
        if (ptr && ptr->opened && !ptr->closed) {
            open.push_back(ptr);
        }
    }
    return open;
}

std::vector<StatePtr> EsdfSearch::getCloseSet() const
{
    std::vector<StatePtr> close;
    for (const auto & item : hm_) {
        const auto & ptr = item.second;
        if (ptr && ptr->closed) {
            close.push_back(ptr);
        }
    }
    return close;
}

std::vector<StatePtr> EsdfSearch::getAllSet() const
{
    std::vector<StatePtr> all;
    for (const auto & item : hm_) {
        const auto & ptr = item.second;
        if (ptr) {
            all.push_back(ptr);
        }
    }
    return all;
}

}  // namespace JPS
