

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <iterator>
#include <limits>
#include <numeric>
#include <queue>
#include <random>
#include <set>
#include <stdexcept>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

using namespace std;

using int64 = long long;
using uint64 = unsigned long long;
using Real = long double;

const int64 INT64_MAX_VALUE = numeric_limits<int64>::max();
const Real REAL_INF = numeric_limits<Real>::infinity();

// ============================================================
// DATA STRUCTURES
// ============================================================

struct Road {
    int u;
    int v;
    int64 weight;
};

struct AdjEdge {
    int to;
    int edgeId;
};

struct Graph {
    int n;
    vector<Road> roads;
    vector<vector<AdjEdge>> adj;

    Graph(int vertexCount, int edgeCapacity)
        : n(vertexCount), adj(vertexCount) {
        roads.reserve(edgeCapacity);
    }

    void addRoad(int u, int v, int64 weight) {
        int id = static_cast<int>(roads.size());

        roads.push_back({u, v, weight});

        // Every road is bidirectional.
        adj[u].push_back({v, id});
        adj[v].push_back({u, id});
    }
};

struct Path {
    vector<int> nodes;
    vector<int> edges;
    Real distance = 0;
};

struct Request {
    int source = 0;
    int destination = 0;
    int64 vehicles = 0;
};

struct Metrics {
    int64 maximumRoadLoad = 0;
    Real averageRoadLoad = 0;
    Real loadVariance = 0;
    Real totalVehicleDistance = 0;
    Real objectiveValue = 0;
};

struct AlgorithmResult {
    vector<vector<Path>> paths;
    vector<vector<int64>> assignment;
    vector<int64> roadLoads;

    Metrics metrics;

    Real runtimeMs = 0;
    int localImprovementIterations = 0;
};

struct TransferPair {
    int fromPath = -1;
    int toPath = -1;

    vector<int> removeEdges;
    vector<int> addEdges;

    int differenceInEdgeCount = 0;
    int changedEdgeCount = 0;
};

struct Move {
    bool exists = false;

    int requestIndex = -1;
    int fromPath = -1;
    int toPath = -1;

    int64 vehicles = 0;
    Real deltaObjective = 0;
};

struct ScenarioConfig {
    int n = 0;
    int m = 0;
    int timeSteps = 0;
    int requestsPerStep = 0;

    int64 minWeight = 1;
    int64 maxWeight = 1;

    int64 minVehicles = 1;
    int64 maxVehicles = 1;
};

struct AverageSummary {
    Real maximumLoadSum = 0;
    Real averageLoadSum = 0;
    Real varianceSum = 0;
    Real distanceSum = 0;
    Real objectiveSum = 0;
    Real runtimeSum = 0;
};

// ============================================================
// RANDOM CONNECTED WEIGHTED GRAPH GENERATION
// ============================================================

static uint64 edgeKey(int u, int v, int n) {
    if (u > v) {
        swap(u, v);
    }

    return static_cast<uint64>(u) *
           static_cast<uint64>(n) +
           static_cast<uint64>(v);
}

Graph generateConnectedGraph(
    int n,
    int m,
    int64 minWeight,
    int64 maxWeight,
    mt19937_64& rng
) {
    Graph graph(n, m);

    unordered_set<uint64> usedEdges;
    usedEdges.reserve(static_cast<size_t>(m));

    uniform_int_distribution<int64> weightDistribution(
        minWeight, maxWeight
    );

    auto tryAddRoad = [&](int u, int v) -> bool {

        if (u == v) {
            return false;
        }

        if (u > v) {
            swap(u, v);
        }

        uint64 key = edgeKey(u, v, n);

        if (!usedEdges.insert(key).second) {
            return false;
        }

        graph.addRoad(
            u,
            v,
            weightDistribution(rng)
        );

        return true;
    };

    // --------------------------------------------------------
    // Step 1: Generate a random spanning tree.
    //
    // A spanning tree contains exactly n-1 roads.
    // Therefore, the generated graph is guaranteed to be
    // connected.
    // --------------------------------------------------------

    vector<int> order(n);

    iota(order.begin(), order.end(), 0);

    shuffle(order.begin(), order.end(), rng);

    for (int i = 1; i < n; ++i) {

        uniform_int_distribution<int> parentDistribution(
            0, i - 1
        );

        int parentIndex = parentDistribution(rng);

        tryAddRoad(
            order[i],
            order[parentIndex]
        );
    }

    // --------------------------------------------------------
    // Step 2: Add extra random roads.
    //
    // Self-loops and duplicate roads are not allowed.
    // --------------------------------------------------------

    if (static_cast<int>(graph.roads.size()) < m) {

        uniform_int_distribution<int> vertexDistribution(
            0, n - 1
        );

        uint64 failures = 0;

        uint64 remaining =
            static_cast<uint64>(m - graph.roads.size());

        uint64 attemptLimit =
            max<uint64>(10000ULL, remaining * 30ULL);

        while (
            static_cast<int>(graph.roads.size()) < m &&
            failures < attemptLimit
        ) {

            int u = vertexDistribution(rng);
            int v = vertexDistribution(rng);

            if (!tryAddRoad(u, v)) {
                ++failures;
            }
        }

        // Dense graph fallback.
        if (static_cast<int>(graph.roads.size()) < m) {

            for (int u = 0;
                 u < n &&
                 static_cast<int>(graph.roads.size()) < m;
                 ++u) {

                for (int v = u + 1;
                     v < n &&
                     static_cast<int>(graph.roads.size()) < m;
                     ++v) {

                    tryAddRoad(u, v);
                }
            }
        }
    }

    if (static_cast<int>(graph.roads.size()) != m) {
        throw runtime_error(
            "Could not generate the requested number of roads."
        );
    }

    return graph;
}

// ============================================================
// RANDOM TRANSPORTATION REQUEST GENERATION
// ============================================================

vector<Request> generateRequests(
    int n,
    int requestCount,
    int64 minVehicles,
    int64 maxVehicles,
    mt19937_64& rng
) {
    vector<Request> requests;

    requests.reserve(requestCount);

    uniform_int_distribution<int> vertexDistribution(
        0, n - 1
    );

    uniform_int_distribution<int64> vehicleDistribution(
        minVehicles, maxVehicles
    );

    for (int i = 0; i < requestCount; ++i) {

        int source = vertexDistribution(rng);
        int destination = vertexDistribution(rng);

        // Ensure source and destination are different.
        if (destination == source) {
            destination = (destination + 1) % n;
        }

        requests.push_back({
            source,
            destination,
            vehicleDistribution(rng)
        });
    }

    return requests;
}

// ============================================================
// DIJKSTRA SHORTEST PATH
//
// Supports temporarily banned vertices and roads.
// These restrictions are required by Yen's algorithm.
// ============================================================

bool dijkstra(
    const Graph& graph,
    int source,
    int destination,
    const vector<char>& bannedNodes,
    const vector<char>& bannedEdges,
    Path& result
) {
    if (source == destination) {

        result.nodes = {source};
        result.edges.clear();
        result.distance = 0;

        return true;
    }

    if (bannedNodes[source] || bannedNodes[destination]) {
        return false;
    }

    vector<Real> dist(graph.n, REAL_INF);

    vector<int> parentNode(graph.n, -1);
    vector<int> parentEdge(graph.n, -1);

    using State = pair<Real, int>;

    priority_queue<
        State,
        vector<State>,
        greater<State>
    > pq;

    dist[source] = 0;

    pq.push({0, source});

    while (!pq.empty()) {
        const State currentState = pq.top();
        pq.pop();

        const Real currentDistance = currentState.first;
        const int u = currentState.second;

        if (currentDistance > dist[u]) {
            continue;
        }

        if (u == destination) {
            break;
        }

        for (const AdjEdge& a : graph.adj[u]) {
            int v = a.to;
            int edgeId = a.edgeId;

            if (bannedNodes[v] || bannedEdges[edgeId]) {
                continue;
            }

            Real newDistance =
                currentDistance +
                static_cast<Real>(graph.roads[edgeId].weight);

            if (newDistance < dist[v]) {

                dist[v] = newDistance;

                parentNode[v] = u;
                parentEdge[v] = edgeId;

                pq.push({newDistance, v});
            }
        }
    }

    if (!isfinite(dist[destination])) {
        return false;
    }

    // Reconstruct the shortest path.
    result.nodes.clear();
    result.edges.clear();

    result.distance = dist[destination];

    int current = destination;

    result.nodes.push_back(current);

    while (current != source) {

        if (parentNode[current] == -1 ||
            parentEdge[current] == -1) {

            return false;
        }

        result.edges.push_back(parentEdge[current]);

        current = parentNode[current];

        result.nodes.push_back(current);
    }

    reverse(result.nodes.begin(), result.nodes.end());

    reverse(result.edges.begin(), result.edges.end());

    return true;
}

// ============================================================
// YEN'S K-SHORTEST LOOPLESS PATHS
//
// Generates up to K shortest paths without repeated vertices.
// ============================================================

struct PathComparator {
    bool operator()(const Path& a, const Path& b) const {

        if (a.distance != b.distance) {
            return a.distance > b.distance;
        }

        return a.edges > b.edges;
    }
};

bool hasSameRootPrefix(
    const Path& path,
    const vector<int>& rootNodes,
    const vector<int>& rootEdges,
    int spurIndex
) {
    if (
        path.nodes.size() <= static_cast<size_t>(spurIndex) ||
        path.edges.size() <= static_cast<size_t>(spurIndex)
    ) {
        return false;
    }

    for (int i = 0; i < spurIndex; ++i) {

        if (path.edges[i] != rootEdges[i]) {
            return false;
        }
    }

    for (int i = 0; i <= spurIndex; ++i) {

        if (path.nodes[i] != rootNodes[i]) {
            return false;
        }
    }

    return true;
}

vector<Path> yenKShortestPaths(
    const Graph& graph,
    int source,
    int destination,
    int K
) {
    vector<Path> acceptedPaths;

    vector<char> noBannedNodes(graph.n, false);

    vector<char> noBannedEdges(
        graph.roads.size(), false
    );

    // First shortest path.
    Path firstPath;

    if (!dijkstra(
            graph,
            source,
            destination,
            noBannedNodes,
            noBannedEdges,
            firstPath
        )) {

        return acceptedPaths;
    }

    acceptedPaths.push_back(firstPath);

    set<vector<int>> acceptedKeys;
    set<vector<int>> queuedKeys;

    acceptedKeys.insert(firstPath.edges);

    priority_queue<
        Path,
        vector<Path>,
        PathComparator
    > candidates;

    // Generate the remaining K-1 paths.
    for (int k = 1; k < K; ++k) {

        const Path& previousPath = acceptedPaths.back();

        // Try every non-destination vertex as a spur node.
        for (
            int spurIndex = 0;
            spurIndex + 1 <
                static_cast<int>(previousPath.nodes.size());
            ++spurIndex
        ) {

            // Root path.
            vector<int> rootNodes(
                previousPath.nodes.begin(),
                previousPath.nodes.begin() + spurIndex + 1
            );

            vector<int> rootEdges(
                previousPath.edges.begin(),
                previousPath.edges.begin() + spurIndex
            );

            vector<char> bannedNodes(graph.n, false);

            vector<char> bannedEdges(
                graph.roads.size(), false
            );

            // Ban earlier vertices in the root path.
            for (int i = 0; i < spurIndex; ++i) {

                bannedNodes[rootNodes[i]] = true;
            }

            // Ban the next edge of every accepted path
            // having the same root prefix.
            for (const Path& accepted : acceptedPaths) {

                if (hasSameRootPrefix(
                        accepted,
                        rootNodes,
                        rootEdges,
                        spurIndex
                    )) {

                    bannedEdges[
                        accepted.edges[spurIndex]
                    ] = true;
                }
            }

            // Find the shortest spur path.
            Path spurPath;

            int spurNode = rootNodes.back();

            if (!dijkstra(
                    graph,
                    spurNode,
                    destination,
                    bannedNodes,
                    bannedEdges,
                    spurPath
                )) {

                continue;
            }

            // Combine root and spur paths.
            Path totalPath;

            totalPath.nodes = rootNodes;

            totalPath.nodes.insert(
                totalPath.nodes.end(),
                spurPath.nodes.begin() + 1,
                spurPath.nodes.end()
            );

            totalPath.edges = rootEdges;

            totalPath.edges.insert(
                totalPath.edges.end(),
                spurPath.edges.begin(),
                spurPath.edges.end()
            );

            totalPath.distance = 0;

            for (int edgeId : totalPath.edges) {

                totalPath.distance +=
                    static_cast<Real>(
                        graph.roads[edgeId].weight
                    );
            }

            // Avoid duplicate paths.
            if (
                acceptedKeys.count(totalPath.edges) == 0 &&
                queuedKeys.count(totalPath.edges) == 0
            ) {

                candidates.push(totalPath);

                queuedKeys.insert(totalPath.edges);
            }
        }

        if (candidates.empty()) {
            break;
        }

        // Accept the shortest remaining candidate.
        Path nextPath = candidates.top();

        candidates.pop();

        acceptedKeys.insert(nextPath.edges);

        acceptedPaths.push_back(move(nextPath));
    }

    return acceptedPaths;
}

// ============================================================
// EXACT POPULATION-VARIANCE CHANGE FOR ADDING A BATCH
//
// Population variance:
//
// Var(L) = S2/m - S1^2/m^2
//
// This calculates the exact variance change, including the
// change in average road load.
// ============================================================

Real varianceChangeWhenAddingBatch(
    const vector<int64>& loads,
    const Path& path,
    int64 batch,
    Real sumRoadLoads
) {
    int m = static_cast<int>(loads.size());

    if (m == 0) {
        return 0;
    }

    Real sumLoadsOnPath = 0;

    for (int edgeId : path.edges) {

        sumLoadsOnPath +=
            static_cast<Real>(loads[edgeId]);
    }

    Real b = static_cast<Real>(batch);

    Real pathLengthInEdges =
        static_cast<Real>(path.edges.size());

    Real deltaS1 =
        b * pathLengthInEdges;

    Real deltaS2 =
        2 * b * sumLoadsOnPath
        + b * b * pathLengthInEdges;

    Real mr = static_cast<Real>(m);

    return deltaS2 / mr
         - (
             2 * sumRoadLoads * deltaS1
             + deltaS1 * deltaS1
           ) / (mr * mr);
}

// ============================================================
// INITIAL TRAFFIC ASSIGNMENT
//
// AKTA:
// Minimize alpha * batch * distance
//          + beta * exact variance change.
//
// GLB:
// Minimize exact variance change.
// Use distance as a tie-breaker.
//
// Both use batches of at most B vehicles.
// ============================================================

void assignBatchesGreedily(
    const vector<Request>& requests,
    AlgorithmResult& result,
    int64 batchLimit,
    Real alpha,
    Real beta,
    bool useDistanceTieBreak
) {
    Real sumRoadLoads = 0;

    for (size_t i = 0; i < requests.size(); ++i) {

        int64 remaining = requests[i].vehicles;

        while (remaining > 0) {

            int64 batch = min(batchLimit, remaining);

            int bestPath = -1;

            Real bestCost = REAL_INF;
            Real bestDistance = REAL_INF;

            for (
                int p = 0;
                p < static_cast<int>(result.paths[i].size());
                ++p
            ) {

                const Path& path = result.paths[i][p];

                Real varianceDelta =
                    varianceChangeWhenAddingBatch(
                        result.roadLoads,
                        path,
                        batch,
                        sumRoadLoads
                    );

                Real cost =
                    alpha *
                    static_cast<Real>(batch) *
                    path.distance
                    + beta * varianceDelta;

                bool better = cost < bestCost;

                if (
                    useDistanceTieBreak &&
                    cost == bestCost &&
                    path.distance < bestDistance
                ) {
                    better = true;
                }

                if (better) {

                    bestCost = cost;
                    bestDistance = path.distance;

                    bestPath = p;
                }
            }

            if (bestPath == -1) {

                throw runtime_error(
                    "A request has no available candidate path."
                );
            }

            // Assign the batch.
            result.assignment[i][bestPath] += batch;

            // Update road loads.
            for (
                int edgeId :
                result.paths[i][bestPath].edges
            ) {

                if (
                    result.roadLoads[edgeId] >
                    INT64_MAX_VALUE - batch
                ) {

                    throw overflow_error(
                        "Road load exceeds int64 range."
                    );
                }

                result.roadLoads[edgeId] += batch;
            }

            // Update total sum of road loads.
            sumRoadLoads +=
                static_cast<Real>(batch) *
                static_cast<Real>(
                    result.paths[i][bestPath].edges.size()
                );

            remaining -= batch;
        }
    }
}

// ============================================================
// PRECOMPUTE ROAD SET DIFFERENCES
//
// For every ordered path pair:
//
// removeEdges = roads in p but not q.
// addEdges    = roads in q but not p.
//
// Shared roads do not change their loads.
// ============================================================

vector<vector<TransferPair>> buildTransferPairs(
    const vector<vector<Path>>& pathsByRequest
) {
    vector<vector<TransferPair>> allPairs(
        pathsByRequest.size()
    );

    for (
        size_t i = 0;
        i < pathsByRequest.size();
        ++i
    ) {

        const auto& paths = pathsByRequest[i];

        int pathCount = static_cast<int>(paths.size());

        vector<vector<int>> sortedEdges(pathCount);

        for (int p = 0; p < pathCount; ++p) {

            sortedEdges[p] = paths[p].edges;

            sort(
                sortedEdges[p].begin(),
                sortedEdges[p].end()
            );
        }

        for (int p = 0; p < pathCount; ++p) {

            for (int q = 0; q < pathCount; ++q) {

                if (p == q) {
                    continue;
                }

                TransferPair pair;

                pair.fromPath = p;
                pair.toPath = q;

                // Roads only in source path.
                set_difference(
                    sortedEdges[p].begin(),
                    sortedEdges[p].end(),

                    sortedEdges[q].begin(),
                    sortedEdges[q].end(),

                    back_inserter(pair.removeEdges)
                );

                // Roads only in destination path.
                set_difference(
                    sortedEdges[q].begin(),
                    sortedEdges[q].end(),

                    sortedEdges[p].begin(),
                    sortedEdges[p].end(),

                    back_inserter(pair.addEdges)
                );

                pair.differenceInEdgeCount =
                    static_cast<int>(pair.addEdges.size())
                    - static_cast<int>(pair.removeEdges.size());

                pair.changedEdgeCount =
                    static_cast<int>(pair.addEdges.size())
                    + static_cast<int>(pair.removeEdges.size());

                allPairs[i].push_back(move(pair));
            }
        }
    }

    return allPairs;
}

// ============================================================
// FIND THE BEST INTEGER TRANSFER
//
// delta(b) = A*b + C*b^2
//
// Since C >= 0, the objective is convex.
//
// Check:
// 1. Feasible endpoints.
// 2. Integers around the continuous minimum.
// ============================================================

int64 findBestIntegerTransfer(
    Real A,
    Real C,
    int64 maximumTransfer,
    Real& bestDelta
) {
    bestDelta = REAL_INF;

    if (maximumTransfer <= 0) {
        return 0;
    }

    vector<int64> candidates;

    auto addCandidate = [&](int64 value) {

        if (
            value < 1 ||
            value > maximumTransfer
        ) {
            return;
        }

        if (
            find(
                candidates.begin(),
                candidates.end(),
                value
            ) == candidates.end()
        ) {

            candidates.push_back(value);
        }
    };

    // Feasible endpoints.
    addCandidate(1);

    addCandidate(maximumTransfer);

    if (C > 0) {

        // Continuous minimum.
        Real optimum = -A / (2 * C);

        if (
            isfinite(optimum) &&
            optimum > 1 &&
            optimum < static_cast<Real>(maximumTransfer)
        ) {

            Real lower = floor(optimum);
            Real upper = ceil(optimum);

            if (
                lower >= 1 &&
                lower <= static_cast<Real>(maximumTransfer)
            ) {

                addCandidate(
                    static_cast<int64>(lower)
                );
            }

            if (
                upper >= 1 &&
                upper <= static_cast<Real>(maximumTransfer)
            ) {

                addCandidate(
                    static_cast<int64>(upper)
                );
            }
        }
    }

    int64 bestB = 0;

    for (int64 b : candidates) {

        Real br = static_cast<Real>(b);

        Real delta =
            A * br + C * br * br;

        if (delta < bestDelta) {

            bestDelta = delta;

            bestB = b;
        }
    }

    return bestB;
}

// ============================================================
// AKTA LOCAL IMPROVEMENT
//
// Uses the exact population-variance objective.
//
// For every request and ordered path pair:
//
// 1. Find the best feasible integer transfer.
// 2. Calculate exact objective change.
// 3. Select the globally best negative change.
// 4. Apply the move and update affected road loads.
//
// Stop after H iterations or when no improvement remains.
// ============================================================

int improveAssignments(
    const vector<Request>& requests,
    AlgorithmResult& result,
    const vector<vector<TransferPair>>& transferPairs,
    int H,
    Real alpha,
    Real beta
) {
    const int m =
        static_cast<int>(result.roadLoads.size());

    if (m == 0) {
        return 0;
    }

    // S1 = sum of all road loads.
    Real sumRoadLoads = 0;

    for (int64 load : result.roadLoads) {

        sumRoadLoads += static_cast<Real>(load);
    }

    int iteration = 0;

    while (iteration < H) {

        Move bestMove;

        Real bestImprovement = 0;

        for (
            size_t i = 0;
            i < requests.size();
            ++i
        ) {

            for (const TransferPair& pair : transferPairs[i]) {

                int p = pair.fromPath;
                int q = pair.toPath;

                int64 maximumTransfer =
                    result.assignment[i][p];

                if (maximumTransfer <= 0) {
                    continue;
                }

                Real addLoadSum = 0;
                Real removeLoadSum = 0;

                for (int edgeId : pair.addEdges) {

                    addLoadSum +=
                        static_cast<Real>(
                            result.roadLoads[edgeId]
                        );
                }

                for (int edgeId : pair.removeEdges) {

                    removeLoadSum +=
                        static_cast<Real>(
                            result.roadLoads[edgeId]
                        );
                }

                Real d = static_cast<Real>(
                    pair.differenceInEdgeCount
                );

                Real k = static_cast<Real>(
                    pair.changedEdgeCount
                );

                Real mr = static_cast<Real>(m);

                Real loadDifference =
                    addLoadSum - removeLoadSum;

                const Path& pathP =
                    result.paths[i][p];

                const Path& pathQ =
                    result.paths[i][q];

                // Exact linear coefficient.
                Real A =
                    alpha * (
                        pathQ.distance - pathP.distance
                    )
                    + beta * (
                        2 * loadDifference / mr
                        - 2 * sumRoadLoads * d / (mr * mr)
                    );

                // Exact quadratic coefficient.
                Real varianceQuadratic =
                    (k * mr - d * d) / (mr * mr);

                if (varianceQuadratic < 0) {
                    varianceQuadratic = 0;
                }

                Real C =
                    beta * varianceQuadratic;

                Real delta;

                int64 transfer =
                    findBestIntegerTransfer(
                        A,
                        C,
                        maximumTransfer,
                        delta
                    );

                // Select the globally best improving move.
                if (
                    transfer > 0 &&
                    delta < bestImprovement
                ) {

                    bestImprovement = delta;

                    bestMove.exists = true;

                    bestMove.requestIndex =
                        static_cast<int>(i);

                    bestMove.fromPath = p;
                    bestMove.toPath = q;

                    bestMove.vehicles = transfer;

                    bestMove.deltaObjective = delta;
                }
            }
        }

        // No negative objective change remains.
        if (!bestMove.exists) {
            break;
        }

        int i = bestMove.requestIndex;

        int p = bestMove.fromPath;
        int q = bestMove.toPath;

        int64 b = bestMove.vehicles;

        // Update path assignments.
        result.assignment[i][p] -= b;

        result.assignment[i][q] += b;

        // Find the precomputed transfer information.
        const TransferPair* selectedPair = nullptr;

        for (const TransferPair& pair : transferPairs[i]) {

            if (
                pair.fromPath == p &&
                pair.toPath == q
            ) {

                selectedPair = &pair;
                break;
            }
        }

        if (selectedPair == nullptr) {

            throw logic_error(
                "Selected local-improvement move was not found."
            );
        }

        // Remove vehicles from source-only roads.
        for (int edgeId : selectedPair->removeEdges) {

            if (result.roadLoads[edgeId] < b) {

                throw logic_error(
                    "Invalid road load during local improvement."
                );
            }

            result.roadLoads[edgeId] -= b;
        }

        // Add vehicles to destination-only roads.
        for (int edgeId : selectedPair->addEdges) {

            if (
                result.roadLoads[edgeId] >
                INT64_MAX_VALUE - b
            ) {

                throw overflow_error(
                    "Road load exceeds int64 range."
                );
            }

            result.roadLoads[edgeId] += b;
        }

        // Update total road load S1.
        sumRoadLoads +=
            static_cast<Real>(b) *
            static_cast<Real>(
                selectedPair->differenceInEdgeCount
            );

        ++iteration;
    }

    return iteration;
}

// ============================================================
// EVALUATION
//
// Calculates:
//
// 1. Maximum road load.
// 2. Average road load.
// 3. Population variance.
// 4. Total vehicle-distance.
// 5. Combined objective value.
// ============================================================

Metrics evaluate(
    const vector<vector<Path>>& paths,
    const vector<vector<int64>>& assignment,
    const vector<int64>& roadLoads,
    Real alpha,
    Real beta
) {
    Metrics metrics;

    const int m =
        static_cast<int>(roadLoads.size());

    if (m > 0) {

        Real sum = 0;

        for (int64 load : roadLoads) {

            metrics.maximumRoadLoad =
                max(metrics.maximumRoadLoad, load);

            sum += static_cast<Real>(load);
        }

        metrics.averageRoadLoad =
            sum / static_cast<Real>(m);

        Real squaredDeviationSum = 0;

        for (int64 load : roadLoads) {

            Real difference =
                static_cast<Real>(load)
                - metrics.averageRoadLoad;

            squaredDeviationSum +=
                difference * difference;
        }

        // Population variance over all roads,
        // including roads with zero traffic.
        metrics.loadVariance =
            squaredDeviationSum / static_cast<Real>(m);
    }

    // Total vehicle-distance.
    for (size_t i = 0; i < paths.size(); ++i) {

        for (size_t p = 0; p < paths[i].size(); ++p) {

            metrics.totalVehicleDistance +=
                static_cast<Real>(assignment[i][p])
                * paths[i][p].distance;
        }
    }

    // Combined AKTA objective.
    metrics.objectiveValue =
        alpha * metrics.totalVehicleDistance
        + beta * metrics.loadVariance;

    return metrics;
}

// ============================================================
// SHORTEST-PATH ROUTING (SPR)
//
// Baseline 1:
//
// Every request uses only its shortest-distance path.
// Traffic congestion is not considered during routing.
// ============================================================

AlgorithmResult runShortestPathRouting(
    const Graph& graph,
    const vector<Request>& requests,
    Real alpha,
    Real beta
) {
    auto start = chrono::steady_clock::now();

    AlgorithmResult result;

    result.paths.resize(requests.size());

    result.assignment.resize(requests.size());

    result.roadLoads.assign(
        graph.roads.size(), 0
    );

    vector<char> noBannedNodes(
        graph.n, false
    );

    vector<char> noBannedEdges(
        graph.roads.size(), false
    );

    for (size_t i = 0; i < requests.size(); ++i) {

        Path shortest;

        if (!dijkstra(
                graph,
                requests[i].source,
                requests[i].destination,
                noBannedNodes,
                noBannedEdges,
                shortest
            )) {

            throw runtime_error(
                "Shortest path not found in a connected graph."
            );
        }

        result.paths[i].push_back(move(shortest));

        result.assignment[i] = {
            requests[i].vehicles
        };

        // Assign all vehicles to the shortest path.
        for (int edgeId : result.paths[i][0].edges) {

            if (
                result.roadLoads[edgeId] >
                INT64_MAX_VALUE - requests[i].vehicles
            ) {

                throw overflow_error(
                    "Road load exceeds int64 range."
                );
            }

            result.roadLoads[edgeId] +=
                requests[i].vehicles;
        }
    }

    result.metrics = evaluate(
        result.paths,
        result.assignment,
        result.roadLoads,
        alpha,
        beta
    );

    auto finish = chrono::steady_clock::now();

    result.runtimeMs =
        chrono::duration<Real, milli>(
            finish - start
        ).count();

    return result;
}

// ============================================================
// GREEDY LOAD-BALANCING ROUTING (GLB)
//
// Baseline 2:
//
// 1. Generate K candidate paths.
// 2. Assign vehicles in batches.
// 3. Choose the path with minimum exact variance increase.
// 4. Use distance as a tie-breaker.
//
// Unlike AKTA, GLB does not perform local improvement.
// ============================================================

AlgorithmResult runGreedyLoadBalancing(
    const Graph& graph,
    const vector<Request>& requests,
    int K,
    int64 B,
    Real alpha,
    Real beta
) {
    auto start = chrono::steady_clock::now();

    AlgorithmResult result;

    result.paths.resize(requests.size());

    result.assignment.resize(requests.size());

    result.roadLoads.assign(
        graph.roads.size(), 0
    );

    // Generate candidate paths.
    for (size_t i = 0; i < requests.size(); ++i) {

        result.paths[i] = yenKShortestPaths(
            graph,
            requests[i].source,
            requests[i].destination,
            K
        );

        if (result.paths[i].empty()) {

            throw runtime_error(
                "No candidate path found for a request."
            );
        }

        result.assignment[i].assign(
            result.paths[i].size(), 0
        );
    }

    // Greedy load balancing.
    //
    // alpha = 0, beta = 1 means that routing decisions
    // minimize only the exact variance increase.
    // Distance breaks equal-cost ties.
    assignBatchesGreedily(
        requests,
        result,
        B,
        0,
        1,
        true
    );

    // Evaluate using the common alpha and beta values.
    result.metrics = evaluate(
        result.paths,
        result.assignment,
        result.roadLoads,
        alpha,
        beta
    );

    auto finish = chrono::steady_clock::now();

    result.runtimeMs =
        chrono::duration<Real, milli>(
            finish - start
        ).count();

    return result;
}

// ============================================================
// PROPOSED ALGORITHM: AKTA
//
// Step 1: Initialization.
// Step 2: Yen's K-shortest candidate paths.
// Step 3: Initial traffic assignment.
// Step 4: Local improvement.
// Step 5: Evaluation.
// Step 6: Return results.
// ============================================================

AlgorithmResult runAKTA(
    const Graph& graph,
    const vector<Request>& requests,
    int K,
    int64 B,
    int H,
    Real alpha,
    Real beta
) {
    auto start = chrono::steady_clock::now();

    AlgorithmResult result;

    result.paths.resize(requests.size());

    result.assignment.resize(requests.size());

    result.roadLoads.assign(
        graph.roads.size(), 0
    );

    // --------------------------------------------------------
    // STEP 1: INITIALIZATION
    // --------------------------------------------------------

    // All road loads and assignments are initialized to zero.
    for (size_t i = 0; i < requests.size(); ++i) {
        result.assignment[i].clear();
    }

    // --------------------------------------------------------
    // STEP 2: CANDIDATE PATH GENERATION
    // --------------------------------------------------------

    for (size_t i = 0; i < requests.size(); ++i) {

        result.paths[i] = yenKShortestPaths(
            graph,
            requests[i].source,
            requests[i].destination,
            K
        );

        if (result.paths[i].empty()) {

            throw runtime_error(
                "No candidate path found for a request."
            );
        }

        result.assignment[i].assign(
            result.paths[i].size(),
            0
        );
    }

    // --------------------------------------------------------
    // STEP 3: INITIAL TRAFFIC ASSIGNMENT
    // --------------------------------------------------------

    assignBatchesGreedily(
        requests,
        result,
        B,
        alpha,
        beta,
        false
    );

    // --------------------------------------------------------
    // Precompute path-pair road differences.
    // --------------------------------------------------------

    vector<vector<TransferPair>> transferPairs =
        buildTransferPairs(result.paths);

    // --------------------------------------------------------
    // STEP 4: LOCAL IMPROVEMENT
    // --------------------------------------------------------

    result.localImprovementIterations =
        improveAssignments(
            requests,
            result,
            transferPairs,
            H,
            alpha,
            beta
        );

    // --------------------------------------------------------
    // STEP 5: EVALUATION
    // --------------------------------------------------------

    result.metrics = evaluate(
        result.paths,
        result.assignment,
        result.roadLoads,
        alpha,
        beta
    );

    // --------------------------------------------------------
    // STEP 6: RETURN RESULTS
    // --------------------------------------------------------

    auto finish = chrono::steady_clock::now();

    result.runtimeMs =
        chrono::duration<Real, milli>(
            finish - start
        ).count();

    return result;
}

// ============================================================
// OUTPUT HELPERS
// ============================================================

void printPath(const Path& path) {

    cout << "vertices: ";

    for (size_t i = 0; i < path.nodes.size(); ++i) {

        if (i) {
            cout << " -> ";
        }

        cout << path.nodes[i] + 1;
    }

    cout << " | roads: [";

    for (size_t i = 0; i < path.edges.size(); ++i) {

        if (i) {
            cout << ", ";
        }

        cout << path.edges[i] + 1;
    }

    cout << "]";
}

// ------------------------------------------------------------
// Print transportation requests for a time step.
// ------------------------------------------------------------

void printRequests(
    const vector<Request>& requests
) {
    cout << "\nTransportation requests:\n";

    if (requests.empty()) {

        cout << "  No requests in this time step.\n";

        return;
    }

    for (size_t i = 0; i < requests.size(); ++i) {

        cout << "  Request " << i + 1
             << ": "
             << requests[i].source + 1
             << " -> "
             << requests[i].destination + 1
             << ", vehicles = "
             << requests[i].vehicles
             << '\n';
    }
}

// ------------------------------------------------------------
// Print paths, assignments, road loads and metrics.
// ------------------------------------------------------------

void printAlgorithmResult(
    const string& algorithmName,
    const Graph& graph,
    const vector<Request>& requests,
    const AlgorithmResult& result
) {
    cout << "\n---------------- "
         << algorithmName
         << " ----------------\n";

    // Assigned paths and vehicle distributions.
    for (size_t i = 0; i < requests.size(); ++i) {

        cout << "Request " << i + 1
             << " ("
             << requests[i].source + 1
             << " -> "
             << requests[i].destination + 1
             << ", total vehicles = "
             << requests[i].vehicles
             << ")\n";

        for (
            size_t p = 0;
            p < result.paths[i].size();
            ++p
        ) {

            cout << "  Path " << p + 1
                 << ": assigned vehicles = "
                 << result.assignment[i][p]
                 << ", distance = "
                 << result.paths[i][p].distance
                 << ", ";

            printPath(result.paths[i][p]);

            cout << '\n';
        }
    }

    // Road-wise traffic loads.
    cout << "Road-wise traffic loads:\n";

    for (size_t e = 0; e < graph.roads.size(); ++e) {

        const Road& road = graph.roads[e];

        cout << "  Road " << e + 1
             << " (" << road.u + 1
             << " - " << road.v + 1
             << ", weight = " << road.weight
             << ")"
             << ": load = "
             << result.roadLoads[e]
             << '\n';
    }

    // Performance metrics.
    cout << "Metrics:\n";

    cout << "  Maximum road load       = "
         << result.metrics.maximumRoadLoad << '\n';

    cout << "  Average road load       = "
         << result.metrics.averageRoadLoad << '\n';

    cout << "  Load variance           = "
         << result.metrics.loadVariance << '\n';

    cout << "  Total vehicle-distance  = "
         << result.metrics.totalVehicleDistance << '\n';

    cout << "  Combined objective F    = "
         << result.metrics.objectiveValue << '\n';

    cout << "  Runtime (ms)            = "
         << result.runtimeMs << '\n';

    if (algorithmName == "AKTA") {

        cout << "  Local-improvement moves = "
             << result.localImprovementIterations
             << '\n';
    }
}

// ============================================================
// COMPARISON OF THREE ROUTING ALGORITHMS
// ============================================================

void printComparison(
    const AlgorithmResult& spr,
    const AlgorithmResult& glb,
    const AlgorithmResult& akta
) {
    cout << "\n================ COMPARISON ================\n";

    cout << left << setw(12) << "Algorithm"
         << right << setw(12) << "MaxLoad"
         << setw(14) << "AvgLoad"
         << setw(14) << "Variance"
         << setw(18) << "Veh-Distance"
         << setw(14) << "Objective"
         << setw(13) << "Time(ms)"
         << '\n';

    auto row = [&](const string& name,
                   const AlgorithmResult& r) {

        cout << left << setw(12) << name
             << right << setw(12)
             << r.metrics.maximumRoadLoad
             << setw(14)
             << r.metrics.averageRoadLoad
             << setw(14)
             << r.metrics.loadVariance
             << setw(18)
             << r.metrics.totalVehicleDistance
             << setw(14)
             << r.metrics.objectiveValue
             << setw(13)
             << r.runtimeMs
             << '\n';
    };

    row("SPR", spr);

    row("GLB", glb);

    row("AKTA", akta);
}

// ============================================================
// SCENARIO-LEVEL AVERAGE RESULTS
// ============================================================

void addToSummary(
    AverageSummary& summary,
    const Metrics& metrics,
    Real runtimeMs
) {
    summary.maximumLoadSum +=
        static_cast<Real>(metrics.maximumRoadLoad);

    summary.averageLoadSum +=
        metrics.averageRoadLoad;

    summary.varianceSum +=
        metrics.loadVariance;

    summary.distanceSum +=
        metrics.totalVehicleDistance;

    summary.objectiveSum +=
        metrics.objectiveValue;

    summary.runtimeSum += runtimeMs;
}

void printScenarioSummary(
    int timeSteps,
    const AverageSummary& spr,
    const AverageSummary& glb,
    const AverageSummary& akta
) {
    if (timeSteps <= 0) {
        return;
    }

    Real count = static_cast<Real>(timeSteps);

    cout << "\n========== AVERAGE OVER "
         << timeSteps
         << " TIME STEPS ==========\n";

    auto row = [&](const string& name,
                   const AverageSummary& s) {

        cout << name << ":\n";

        cout << "  Mean per-step maximum load = "
             << s.maximumLoadSum / count << '\n';

        cout << "  Mean per-step average load = "
             << s.averageLoadSum / count << '\n';

        cout << "  Mean per-step variance     = "
             << s.varianceSum / count << '\n';

        cout << "  Mean vehicle-distance      = "
             << s.distanceSum / count << '\n';

        cout << "  Mean objective             = "
             << s.objectiveSum / count << '\n';

        cout << "  Mean runtime (ms)          = "
             << s.runtimeSum / count << '\n';
    };

    row("SPR", spr);

    row("GLB", glb);

    row("AKTA", akta);
}

// ============================================================
// INPUT VALIDATION
// ============================================================

void validateScenario(
    const ScenarioConfig& s,
    int scenarioNumber
) {
    if (s.n < 2) {

        throw runtime_error(
            "Scenario " + to_string(scenarioNumber) +
            ": n must be at least 2."
        );
    }

    if (s.m < s.n - 1) {

        throw runtime_error(
            "Scenario " + to_string(scenarioNumber) +
            ": a connected graph needs m >= n - 1."
        );
    }

    uint64 maxEdges =
        static_cast<uint64>(s.n) *
        static_cast<uint64>(s.n - 1) / 2ULL;

    if (static_cast<uint64>(s.m) > maxEdges) {

        throw runtime_error(
            "Scenario " + to_string(scenarioNumber) +
            ": m exceeds the number of possible simple roads."
        );
    }

    if (
        s.timeSteps <= 0 ||
        s.requestsPerStep < 0
    ) {

        throw runtime_error(
            "Scenario " + to_string(scenarioNumber) +
            ": invalid time-step or request count."
        );
    }

    if (
        s.minWeight <= 0 ||
        s.maxWeight < s.minWeight
    ) {

        throw runtime_error(
            "Scenario " + to_string(scenarioNumber) +
            ": invalid road-weight range."
        );
    }

    if (
        s.minVehicles <= 0 ||
        s.maxVehicles < s.minVehicles
    ) {

        throw runtime_error(
            "Scenario " + to_string(scenarioNumber) +
            ": invalid vehicle-count range."
        );
    }
}

// ============================================================
// MAIN FUNCTION
// ============================================================

int main() {

    ios::sync_with_stdio(false);

    cin.tie(nullptr);

    try {

        // ----------------------------------------------------
        // READ NUMBER OF NETWORK SCENARIOS
        // ----------------------------------------------------

        int scenarioCount;

        cin >> scenarioCount;

        if (!cin || scenarioCount <= 0) {

            throw runtime_error(
                "The number of scenarios must be positive."
            );
        }

        // ----------------------------------------------------
        // READ COMMON AKTA PARAMETERS
        // ----------------------------------------------------

        int K;
        int64 B;

        Real alpha;
        Real beta;

        int H;

        uint64 seed;

        cin >> K >> B >> alpha >> beta >> H >> seed;

        if (
            !cin ||
            K <= 0 ||
            B <= 0 ||
            H < 0 ||
            !isfinite(alpha) ||
            !isfinite(beta) ||
            alpha < 0 ||
            beta < 0
        ) {

            throw runtime_error(
                "Invalid common algorithm parameters."
            );
        }

        // ----------------------------------------------------
        // READ NETWORK SCENARIOS
        // ----------------------------------------------------

        vector<ScenarioConfig> scenarios(
            scenarioCount
        );

        for (
            int i = 0;
            i < scenarioCount;
            ++i
        ) {

            ScenarioConfig& s = scenarios[i];

            cin >> s.n
                >> s.m
                >> s.timeSteps
                >> s.requestsPerStep
                >> s.minWeight
                >> s.maxWeight
                >> s.minVehicles
                >> s.maxVehicles;

            if (!cin) {

                throw runtime_error(
                    "Could not read scenario configuration."
                );
            }

            validateScenario(s, i + 1);
        }

        // ----------------------------------------------------
        // INITIALIZE RANDOM NUMBER GENERATOR
        // ----------------------------------------------------

        mt19937_64 rng(seed);

        cout << fixed << setprecision(6);

        cout << "AKTA MULTI-TIME-STEP SIMULATION\n";

        cout << "K = " << K
             << ", B = " << B
             << ", alpha = " << alpha
             << ", beta = " << beta
             << ", H = " << H
             << ", seed = " << seed
             << '\n';

        cout << "Objective: F = alpha * total vehicle-distance"
             << " + beta * population variance of road loads.\n";

        // ====================================================
        // OUTER LOOP: NETWORK SCENARIOS
        // ====================================================

        for (
            int scenarioIndex = 0;
            scenarioIndex < scenarioCount;
            ++scenarioIndex
        ) {

            const ScenarioConfig& config =
                scenarios[scenarioIndex];

            cout << "\n\n##################################################\n";

            cout << "SCENARIO "
                 << scenarioIndex + 1
                 << '\n';

            cout << "##################################################\n";

            cout << "Vertices = " << config.n
                 << ", roads = " << config.m
                 << ", time steps = " << config.timeSteps
                 << ", requests per step = "
                 << config.requestsPerStep
                 << '\n';

            // ------------------------------------------------
            // TASK 1: RANDOM GRAPH GENERATION
            // ------------------------------------------------

            auto graphStart =
                chrono::steady_clock::now();

            Graph graph = generateConnectedGraph(
                config.n,
                config.m,
                config.minWeight,
                config.maxWeight,
                rng
            );

            auto graphFinish =
                chrono::steady_clock::now();

            Real graphGenerationMs =
                chrono::duration<Real, milli>(
                    graphFinish - graphStart
                ).count();

            cout << "Graph generation time (ms) = "
                 << graphGenerationMs << '\n';

            cout << "Generated undirected weighted roads:\n";

            for (
                size_t e = 0;
                e < graph.roads.size();
                ++e
            ) {

                const Road& road = graph.roads[e];

                cout << "  Road " << e + 1
                     << ": "
                     << road.u + 1
                     << " " << road.v + 1
                     << " weight = "
                     << road.weight
                     << '\n';
            }

            // ------------------------------------------------
            // INITIALIZE SCENARIO SUMMARIES
            // ------------------------------------------------

            AverageSummary sprSummary;
            AverageSummary glbSummary;
            AverageSummary aktaSummary;

            // =================================================
            // OUTER SIMULATION LOOP: TIME STEPS
            // =================================================

            for (
                int time = 1;
                time <= config.timeSteps;
                ++time
            ) {

                cout << "\n\n========== TIME STEP "
                     << time
                     << " ==========\n";

                // ---------------------------------------------
                // Generate new transportation requests.
                // ---------------------------------------------

                vector<Request> requests =
                    generateRequests(
                        config.n,
                        config.requestsPerStep,
                        config.minVehicles,
                        config.maxVehicles,
                        rng
                    );

                // Check total vehicle count for overflow.
                int64 totalVehicles = 0;

                for (const Request& request : requests) {

                    if (
                        totalVehicles >
                        INT64_MAX_VALUE - request.vehicles
                    ) {

                        throw overflow_error(
                            "Total vehicles in a time step "
                            "exceed int64 range."
                        );
                    }

                    totalVehicles += request.vehicles;
                }

                printRequests(requests);

                // ---------------------------------------------
                // TASK 5: RUN ALL THREE ALGORITHMS
                //
                // The same graph and the exact same requests
                // are supplied to each algorithm.
                // ---------------------------------------------

                AlgorithmResult spr =
                    runShortestPathRouting(
                        graph,
                        requests,
                        alpha,
                        beta
                    );

                AlgorithmResult glb =
                    runGreedyLoadBalancing(
                        graph,
                        requests,
                        K,
                        B,
                        alpha,
                        beta
                    );

                AlgorithmResult akta =
                    runAKTA(
                        graph,
                        requests,
                        K,
                        B,
                        H,
                        alpha,
                        beta
                    );

                // ---------------------------------------------
                // TASK 3: REPORT ASSIGNMENTS AND ROAD LOADS
                // ---------------------------------------------

                printAlgorithmResult(
                    "SPR",
                    graph,
                    requests,
                    spr
                );

                printAlgorithmResult(
                    "GLB",
                    graph,
                    requests,
                    glb
                );

                printAlgorithmResult(
                    "AKTA",
                    graph,
                    requests,
                    akta
                );

                // ---------------------------------------------
                // TASK 5: PERFORMANCE COMPARISON
                // ---------------------------------------------

                printComparison(
                    spr,
                    glb,
                    akta
                );

                // ---------------------------------------------
                // Accumulate scenario-level metrics.
                // ---------------------------------------------

                addToSummary(
                    sprSummary,
                    spr.metrics,
                    spr.runtimeMs
                );

                addToSummary(
                    glbSummary,
                    glb.metrics,
                    glb.runtimeMs
                );

                addToSummary(
                    aktaSummary,
                    akta.metrics,
                    akta.runtimeMs
                );
            }

            // =================================================
            // FINAL SCENARIO SUMMARY
            // =================================================

            printScenarioSummary(
                config.timeSteps,
                sprSummary,
                glbSummary,
                aktaSummary
            );
        }

        cout << "\nSIMULATION FINISHED SUCCESSFULLY.\n";
    }
    catch (const exception& error) {

        cerr << "ERROR: "
             << error.what()
             << '\n';

        return 1;
    }

    return 0;
}
