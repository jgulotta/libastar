/*
 * This program reads in two text files which are expected to begin
 * with a line containing only numerical values, which are the
 * ordered values of the tiles in an n-puzzle,
 * where n = k^2 - 1 and k lies in [2, inf).
 *
 * The program then solves this puzzle using three different
 * methods and records the process to a file.
 */

#include <algorithm>
#include <chrono>
#include <iostream>
#include <fstream>
#include <sstream>
#include <cmath>
#include <cstdlib>
#include "Puzzle.h"
#include "astar/solver.h"

using std::abs;
using std::cout;
using std::cerr;
using std::endl;
using std::ofstream;

using clock_type = std::chrono::high_resolution_clock;

/*
 * These are the three examined heuristic functions.
 * displaced(...) counts how many tiles are out of place
 * manhattan(...) determines how far away each tile is from
 * its correct location, as specified in the solution.
 * sumdisman(...) simply sums displaced(...) and manhattan(...).
 */
int displaced(const Puzzle& state, const Puzzle& solution) {
    int result = 0;

    for(auto it1 = begin(state.tiles), it2 = begin(solution.tiles); it1 != end(state.tiles); it1++, it2++) {
        if(*it1 != 0 && *it1 != *it2) {
            result++;
        }
    }

    return result;
}

int manhattan(const Puzzle& state, const Puzzle& solution) {
    int result = -1;

    /*
     * These loops compare two vectors piece by piece,
     * finding the row and column indeces for corresponding
     * pieces, and adding the magnitudes of the differences.
     *
     * This is presently O(n^2).
     * TODO: investigate O(n) solution
     */
    int i = 0;
    size_t width = state.tiles.size();
    for(auto it1 = begin(state.tiles); it1 != end(state.tiles); it1++, i++) {
        if(*it1 != 0) {

            int j = 0;
            for(auto it2 = begin(solution.tiles); *it1 != *it2; it2++, j++);

            int r1 = i / width;
            int r2 = j / width;
            int c1 = i % width;
            int c2 = j % width;

            result += abs(r2 - r1) + abs(c2 - c1);
        }
    }

    return result;
}

int sumdisman(const Puzzle& state, const Puzzle& solution) {
    return displaced(state, solution) + manhattan (state, solution);
}

static const Tile empty = 0;

/**
 * distance from where the given tile is to where it should be
 */
int nilssons_distance(const Tile& t, int i) {
    int r1 = i / 3, c1 = i % 3, r2 = -1, c2 = -1;

    switch (t.value) {
        case 1: r2 = 0; c2 = 0; break;
        case 2: r2 = 0; c2 = 1; break;
        case 3: r2 = 0; c2 = 2; break;
        case 4: r2 = 1; c2 = 2; break;
        case 5: r2 = 2; c2 = 2; break;
        case 6: r2 = 2; c2 = 1; break;
        case 7: r2 = 2; c2 = 0; break;
        case 8: r2 = 1; c2 = 0; break;
    }

    return abs(r2 - r1) + abs(c2 - c1);
}

/**
 * index of the next tile
 */
int nilssons_successor(const Tile& t) {
    switch (t.value) {
        case 1: return 1;
        case 2: return 2;
        case 3: return 5;
        case 4: return 8;
        case 5: return 7;
        case 6: return 6;
        case 7: return 3;
        case 8: return 0;
        default: throw "called function for Nilsson's sequence score with value outside the valid [1,8] range";
    }
}


/**
 * Nilsson's sequence score
 * Assumes a 3x3 grid with goal state
 * 1 2 3
 * 8 0 4
 * 7 6 5
 */
int nilssons(const Puzzle& state, const Puzzle&) {
    int score = 0;

    const auto& tiles = state.tiles;

    // top row and middle left
    for(int i = 0; i < 4; ++i) {
        const auto& t = tiles[i];
        if (t != empty) { 
            score += nilssons_distance(t, i);
            score += 6 * (tiles[nilssons_successor(t)].value != ((t.value + 1) % 8));
        }
    }

    // center
    if (tiles[4] != empty) { 
        score += 3 + nilssons_distance(tiles[4], 4);
    }

    // middle right and bottom row
    for(int i = 5; i < 9; ++i) {
        const auto& t = tiles[i];
        if (t != empty) { 
            score += nilssons_distance(t, i);
            score += 6 * (tiles[nilssons_successor(t)].value != (t.value + 1));
        }
    }

    return score;
}

template<class T, class Gen, class Dist, class Est>
clock_type::rep time_solution(AStarSolver<T,Gen,Dist,Est>& solver) {
    clock_type::time_point start = clock_type::now();
    solver.solve();
    clock_type::time_point end = clock_type::now();
    return std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
}

int main (int argc, char **argv) {
    if(argc < 3) {
        cerr << "Usage: " << argv[0] << " start goal\n";
        exit(1);
    }

    Puzzle to_solve(argv[1]);
    Puzzle solution(argv[2]);

    auto generator = [](std::vector<Puzzle>& v, const Puzzle& p) { return p.get_neighbors(v); };
    auto distance = [](const Puzzle&, const Puzzle&) { return 1; };

    clock_type::rep total = 0;
    size_t reps = 100000;
    for (size_t i = 0; i < reps; ++i) {
        auto solver = make_solver(to_solve, solution, generator, distance, nilssons);
        total += time_solution(solver);
    }
    cout << "Total microseconds: " << total << endl;
    cout << "Average microseconds: " << static_cast<double>(total) / reps << endl;

    return 0;
}

