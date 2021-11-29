# Implementation of Online Learning Multi-path Selection Framework (OLMS)

## Kernel module ##

- `mptcp_olms.c` contains the kernel module implementation 
  - use read, write, ioctl to interact with user program.
- `olms-helper.h` defines the useful data structures in both kernel space and
  user space.



## User program ##

### Requirement ###

- Install the GNU Linear Programming Kit `glpk`
  ([link](https://www.gnu.org/software/glpk/))

### Compilation ###

- `cmake` using C++11 standards;
- In the project directory (the directory that has the file `CmakeLists.txt`),
 run the following command to compile this project. 

  ```bash
  cmake . && cmake --build . -- -j4
  ```

### Testing ###

After compilation, there should be a binary file `multi-path-selection` in the
project directory.

- To test the program, in the project directory, run

  ```bash
  ./multi-path-selection
  ```

  Results will be stored in the file `pathLog.txt` in the project directory.

- To get help, run

  ```bash
  ./multi-path-selection -h
  ```
 
#### (OLMS) CONMP-TS algorithm description ####

- CONMP-TS: select paths using constrained Thompson sampling. In the file
  `src/policy/policy_conmpts.hpp`,

  - `solveConTSLP(A, b, c, h, x)` solves the LP and saves the solution (current
    optimal selection vector) to `x`.
  - `selectNextPaths(uint M)` generates posterior estimations and select `M`
    paths with the current optimal selection vector using the `dependentRouding`
    algorithm (implemented in `src/bandit/bandit_utils.hpp`).
  - `updateState(paths, pathMeasurements)` does the Bernoulli trials using the
    `pathMeasurements` of the `paths` and update the times of success trials.
  - `computeOracleLatency(r, b, M, h)` uses the averages of the measurements to compute
    the optimal selection vector. (This function is not part of the ConMPTS
    algorithm. The function is only useful for logging.).

### Additional Multi-armed Bandit Algorithms ###

- EXP3M: select paths (arms) using exponential weight. See the implementation in
  `src/policy/policy_exp3m.hpp`.
- MP-TS: select paths using simple sampling. Details in
  `src/policy/policy_mpts.hpp`
- KL-UCB: select paths with the highest UCB indices. Details in
  `src/policy/policy_klucb.hpp`
- RANDOM: select paths uniformly randomly. Details in
  `src/policy/policy_random.hpp`
