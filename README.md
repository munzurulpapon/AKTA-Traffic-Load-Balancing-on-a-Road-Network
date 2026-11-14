

# AKTA: Traffic Load Balancing on a Road Network

**Course:** CSE 2202, Algorithm Analysis and Design Laboratory (Lab Test 04 A)
**File:** `Final.cpp` (single file, standard C++, no external libraries)

## What the program does

1. Generates a random **connected, undirected, weighted** road graph (a random spanning tree plus extra random roads).
2. At every time step, generates random transportation requests `(vehicles, source, destination)`.
3. Runs three routing methods on the **same graph and same requests**:
   - **SPR**: Shortest-Path Routing. All vehicles of a request use its one shortest path.
   - **GLB**: Greedy Load-Balancing. K shortest paths (Yen), vehicles assigned in batches to the path with the smallest variance increase.
   - **AKTA** (proposed): Yen K shortest paths + batch assignment (cost = alpha * distance + beta * variance change) + local improvement (moving vehicles between paths, at most H moves).
4. Prints, for every time step: the requests, the assigned paths, the vehicles on each path, the load of every road, the metrics, and a comparison table. At the end of each scenario it prints the average over all time steps.

**Metrics:** maximum road load, average road load, load variance, total vehicle-distance, and the objective `F = alpha * total vehicle-distance + beta * load variance`, plus runtime in ms.

## Compile and run

```bash
g++ -O2 -std=c++17 -o akta Final.cpp
./akta < input.txt > output.txt       # Linux / macOS
akta.exe < input.txt > output.txt     # Windows
```

(With MinGW on Windows use `g++ -O2 -std=c++17 -o akta.exe Final.cpp`.)

## Input format

```
S                                  number of scenarios (S >= 1)
K B alpha beta H seed              common AKTA parameters
n m T R wMin wMax vMin vMax        one line per scenario (S lines)
```

| Symbol | Meaning |
|---|---|
| S | number of network scenarios |
| K | number of candidate paths per request (Yen) |
| B | batch size (vehicles assigned at a time) |
| alpha | weight of total distance in F |
| beta | weight of load variance in F |
| H | maximum number of local-improvement moves |
| seed | random seed (same seed gives the same graph and requests on the same compiler) |
| n, m | number of intersections and roads (needs `n - 1 <= m <= n(n-1)/2`) |
| T | number of time steps |
| R | requests per time step |
| wMin, wMax | range of road lengths |
| vMin, vMax | range of vehicles per request |

## Sample input

```
1
5 4 0.5 10 10 20261110
10 20 2 5 1 20 1 10
```

This means: 1 scenario, K = 5, B = 4, alpha = 0.5, beta = 10, H = 10, seed = 20261110. The network has 10 intersections and 20 roads, with 2 time steps and 5 requests per step. Road lengths are 1 to 20 and vehicles per request are 1 to 10.

The input used for the main report results (3 scenarios):

```
3
5 4 0.5 10 10 20261109
30 60 5 15 1 20 1 10
50 120 5 25 1 20 1 10
80 200 5 40 1 20 1 10
```

## Sample output (shortened)

The full output also lists every generated road, every request, all paths and all road loads for SPR, GLB and AKTA. Only the important parts are shown here.

```
AKTA MULTI-TIME-STEP SIMULATION
K = 5, B = 4, alpha = 0.500000, beta = 10.000000, H = 10, seed = 20261110
Objective: F = alpha * total vehicle-distance + beta * population variance of road loads.

SCENARIO 1
Vertices = 10, roads = 20, time steps = 2, requests per step = 5

========== TIME STEP 1 ==========
Transportation requests:
  Request 1: 2 -> 7, vehicles = 3
  Request 2: 7 -> 8, vehicles = 10
  Request 3: 9 -> 10, vehicles = 9
  Request 4: 3 -> 6, vehicles = 8
  Request 5: 3 -> 7, vehicles = 10
  ...
================ COMPARISON ================
Algorithm        MaxLoad       AvgLoad      Variance      Veh-Distance     Objective     Time(ms)
SPR                   13      3.050000     20.047500        348.000000    374.475000     0.000000
GLB                    9      4.350000      8.627500        696.000000    434.275000     0.972000
AKTA                  10      3.350000     11.227500        460.000000    342.275000     1.992000

========== TIME STEP 2 ==========
  ...
================ COMPARISON ================
Algorithm        MaxLoad       AvgLoad      Variance      Veh-Distance     Objective     Time(ms)
SPR                   16      2.700000     18.410000        400.000000    384.100000     0.000000
GLB                    6      3.100000      5.090000        520.000000    310.900000     1.006000
AKTA                   7      2.850000      4.827500        476.000000    286.275000     2.000000

========== AVERAGE OVER 2 TIME STEPS ==========
SPR:   max load 14.5, avg load 2.875, variance 19.2288, distance 374.0, objective 379.2875
GLB:   max load  7.5, avg load 3.725, variance  6.8588, distance 608.0, objective 372.5875
AKTA:  max load  8.5, avg load 3.100, variance  8.0275, distance 468.0, objective 314.2750

SIMULATION FINISHED SUCCESSFULLY.
```

**How to read it:** SPR has the shortest distance but the highest load and variance. GLB has the lowest variance but the longest distance. AKTA keeps the load low and the distance moderate, so it has the lowest objective F in both time steps.

## Reading the path lines

```
Request 1 (2 -> 7, total vehicles = 3)
  Path 1: assigned vehicles = 2, distance = 14.0, vertices: 2 -> 8 -> 7 | roads: [3, 18]
```

The request has 3 vehicles. Path 1 gets 2 of them. It has length 14, goes through intersections 2, 8, 7 and uses roads number 3 and 18. The numbers of vehicles on all paths of a request add up to the total.

## Notes

- Random numbers use `mt19937_64` with the given seed. The distributions in the C++ standard library differ between compilers (for example GCC and MSVC), so the same seed can give different random graphs and requests on different compilers. The comparison pattern between SPR, GLB and AKTA stays the same.
- Runtime values (ms) depend on the computer.
- Parameter ranges used in the report: K = 5, B = 4, alpha = 0.5, beta = 10, H = 10.
- If beta / alpha is 1 or less, AKTA behaves almost like SPR; a larger beta / alpha gives more load balancing and more distance.
