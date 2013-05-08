/*
 * This program reads in two text files which are expected to begin
 * with a line containing only numerical values, which are the
 * ordered values of the tiles in an n-puzzle,
 * where n = k^2 - 1 and k lies in [2, inf).
 *
 * The program then solves this puzzle using three different
 * methods and records the process to a file.
 */

#include <chrono>
#include <iostream>
#include <fstream>
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

//float nilssons(const Puzzle& state, const Puzzle& solution) {
    //return 0;
//}

template<class T, class G, class H>
clock_type::rep time_solution(AStarSolver<T,G,H>& solver) {
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
    size_t reps = 1;
    for (size_t i = 0; i < reps; ++i) {
        auto solver = make_solver(to_solve, solution, generator, distance, manhattan);
        total += time_solution(solver);
    }
    cout << "Total microseconds: " << total << endl;
    cout << "Average microseconds: " << static_cast<double>(total) / reps << endl;

    return 0;
}

