#pragma once

#include <iostream>
#include <memory>
#include <queue>
#include <set>
#include <type_traits>
#include <utility>
#include <vector>

#include "heapset.h"

/*
 * This class finds the best path from the starting state to the goal state
 * given a way to get neighbor states, a way to measure distance from the last state,
 * and a way to estimate cost to the goal state.
 *
 * Generator must be a callable conforming to `...(std::vector<T>, const T&)`.
 * Distance and Estimate must both conform to `...(const T&, const T&)`.
 * The return value of Generator is not used and may be `void`.
 * The return types of Distance and Estimate need not be the same,
 * but they must support `operator+` in the form `Distance(...) + Estimate(...)`.
 */

template<class T, class Generator, class Distance, class Estimate>
class AStarSolver
{
    public:
        using G = std::result_of_t<Distance(const T&, const T&)>;
        using H = std::result_of_t<Estimate(const T&, const T&)>;

        AStarSolver(T to_solve, T goal_state, Generator&& g, Distance&& d, Estimate&& e);

        std::ostream& print_solution(std::ostream& out = std::cout) const;
        /*
         * Attempts to solve the puzzle and returns true if it was able, false otherwise
         */
        bool solve();

    private:
        struct Node
        {
                Node* prev_;
                T state_;
                G distance_;
                H estimate_;
                decltype(distance_ + estimate_) cost_;

                Node(const AStarSolver& s, T&& state, Node* previous = nullptr);
        };


        struct ByCost {
            // using greater than creates a min-heap
            bool operator()(const Node& l, const Node& r) const {
                return l.cost_ > r.cost_;
            }
        };

        struct ByState {
            bool operator()(const Node& l, const Node& r) const {
                return l.state_ < r.state_;
            }
        };

        T goal_;
        Generator generator_func_;
        Distance distance_func_;
        Estimate cost_func_;
        Node* last_;
        HeapSet<Node,ByState,ByCost> states_;
};

template<class T, class Gen, class Dist, class Est>
AStarSolver<T,Gen,Dist,Est>::Node::Node(const AStarSolver& as, T&& s, Node* p) :
    prev_(p), state_(std::forward<T>(s)),
    distance_(p ? (p->distance_ + as.distance_func_(p->state_, state_)) : G{}),
    estimate_(as.cost_func_(state_, as.goal_)),
    cost_(distance_ + estimate_)
{
}

template<class T, class Gen, class Dist, class Est>
AStarSolver<T,Gen,Dist,Est>::AStarSolver(T s, T g, Gen&& gen, Dist&& d, Est&& e) :
    goal_(std::move(g)), generator_func_(std::forward<Gen>(gen)), distance_func_(std::forward<Dist>(d)), cost_func_(std::forward<Est>(e))
{
    states_.emplace(*this,std::move(s));
}

template<class T, class Gen, class Dist, class Est>
std::ostream& AStarSolver<T,Gen,Dist,Est>::print_solution(std::ostream& o) const
{
    if (last_)
    {
        o << "Solution:\n";

        auto p = last_;
        size_t step = 0;
        do {
            o << "Step: " << ++step << '\n';
            o << "Estimate: " << p->cost_ << '\n';
            o << "State:\n" << p->state_ << '\n';
        } while((p = p->prev_));

    }
    else
    {
        o << "No solution found.\n";
    }
    return o;
}

template<class T, class Gen, class Dist, class Est>
bool AStarSolver<T,Gen,Dist,Est>::solve()
{
    std::vector<T> neighbors;

    while (!states_.empty()) {
        auto pnode = states_.pop();

        if (pnode->state_ == goal_) {
            last_ = pnode;
            return true;
        }

        generator_func_(neighbors, pnode->state_);

        for (auto& n : neighbors) {
            states_.emplace(*this, std::move(n), pnode);
        }

        neighbors.clear();
    }

    return false;
}

template<class T, class Gen, class Dist, class Est>
auto make_solver(T&& start, T&& goal, Gen&& generator, Dist&& distance, Est&& estimate) {
    return AStarSolver<std::remove_reference_t<T>, Gen, Dist, Est>
               (std::forward<T>(start),
                std::forward<T>(goal),
                std::forward<Gen>(generator),
                std::forward<Dist>(distance),
                std::forward<Est>(estimate));
}
