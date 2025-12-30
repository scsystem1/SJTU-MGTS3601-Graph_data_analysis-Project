#include <iostream>
#include <fstream>
#include <vector>
#include <set>
#include <unordered_set>
#include <unordered_map>
#include <algorithm>
#include <chrono>
#include <tuple>
#include <queue>
using namespace std;
using namespace std::chrono;

class UndirectedGraph {
public:
    int vertex_num;
    vector<vector<int>> adj_list;

    UndirectedGraph(const vector<pair<int, int>>& edge_list) {
        vertex_num = 0;
        for (auto& edge : edge_list) {
            vertex_num = max(vertex_num, max(edge.first, edge.second) + 1);
        }
        adj_list.resize(vertex_num);
        for (auto& edge : edge_list) {
            int u = edge.first;
            int v = edge.second;
            if (u == v) continue; // skip self-loops
            adj_list[u].push_back(v);
            adj_list[v].push_back(u);
        }
    }
};

bool check_result(const vector<int>& prediction, const vector<int>& ground_truth) {
    return prediction == ground_truth;
}

vector<pair<int, int>> load_edge_list(const string& filename) {
    vector<pair<int, int>> edges;
    ifstream infile(filename);
    int u, v;
    while (infile >> u >> v) {
        if (u == v) continue;
        if (u > v) swap(u, v);
        edges.emplace_back(u, v);
    }
    infile.close();
    sort(edges.begin(), edges.end());
    edges.erase(unique(edges.begin(), edges.end()), edges.end());
    return edges;
}

vector<int> load_ground_truth(const string& filename) {
    vector<int> result;
    ifstream infile(filename);
    int val;
    while (infile >> val) {
        result.push_back(val);
    }
    infile.close();
    return result;
}

struct SuperNode {
    int id;
    unordered_set<int> V;
    int truss;
    bool operator==(const SuperNode& other) const {
        return id == other.id;
    }
};

struct SuperNodeHash {
    size_t operator()(const SuperNode& sn) const {
        return hash<int>()(sn.id);
    }
};

struct Comparebytruss {
    bool operator()(const tuple<int,int,int>& a, const tuple<int,int,int>& b) const {
        return get<2>(a) < get<2>(b);
    }
};

vector<int> findintersection(const vector<int>& a, const vector<int>& b) {
    vector<int> result;
    unordered_set<int> set_a(a.begin(), a.end());
    for (int val : b) {
        if(set_a.contains(val)) {
            result.push_back(val);
        }
    }
    return result;
}

int findintersectionnum(const vector<int>& a, const vector<int>& b) {
    int result = 0;
    unordered_set<int> set_a(a.begin(), a.end());
    for (int val : b) {
        if(set_a.contains(val)) {
            result++;
        }
    }
    return result;
}

bool isConnected(const unordered_map<int, unordered_map<int,int>>& super_adj_list, int src, int dst){
    unordered_set<int> visited;
    queue<int> q;
    visited.insert(src);
    q.push(src);

    while (!q.empty()) {
        int u = q.front(); q.pop();
        // 遍历所有邻居
        auto it = super_adj_list.find(u);
        if (it == super_adj_list.end()) continue;
        for (const auto& [nei, truss] : it->second) {
            if (visited.count(nei)) continue;
            if (nei == dst) {
                return true;
            }
            visited.insert(nei);
            q.push(nei);
        }
    }
    return false;
}

int Num_Connected_Component(const unordered_map<int, unordered_map<int, pair<int,bool>>>& adj_list) {
    unordered_set<int> visited;
    int count = 0;

    for (const auto& [node, neighbors] : adj_list) {
        if (neighbors.empty()) continue;           // 避免误统计孤点
        if (visited.count(node)) continue;

        // 以当前未访问节点为起点做 BFS
        count++;
        queue<int> q;
        q.push(node);
        visited.insert(node);

        while (!q.empty()) {
            int u = q.front(); q.pop();
            for (const auto& [nei, _] : adj_list.at(u)) {
                if (!visited.count(nei)) {
                    visited.insert(nei);
                    q.push(nei);
                }
            }
        }
    }
    return count;
}

class GCT_INDEX {
public:
    //存储每个节点的ego_network，具体为 {w, {u, {trussness（计算trussness前存support）, is_calculated_trussness(用于truss decomposition)}}}
    vector<unordered_map<int, unordered_map<int,pair<int,bool>>>> ego_network;
    //对每个节点ego_network按trussness构建优先队列（最大堆）
    vector<priority_queue<tuple<int,int,int>, vector<tuple<int,int,int>>, Comparebytruss>> truss_queue;
    //存储每个节点ego_network中的不同trussness的超点个数
    vector<vector<int>> node_truss_num;
    //存储每个节点ego_network中的不同trussness的超边个数
    vector<vector<int>> edge_truss_num;
    void build_egonetwork(const UndirectedGraph& G);
    void compute_truss();
    void build_supergraph_index();
    vector<int> compute_diversirty(int k);
    vector<int> compute_diversirty_naive(int k);
};

void GCT_INDEX::build_egonetwork(const UndirectedGraph& G) {
    ego_network.resize(G.vertex_num);
    node_truss_num.resize(G.vertex_num);
    edge_truss_num.resize(G.vertex_num);
    //遍历所有边（u，v），并将其加入到u，v共同邻居的ego_network中
    for(int i = 0; i < G.vertex_num; i++) {
        for (int j : G.adj_list[i]) {
            if(i < j) {
                vector<int> W = findintersection(G.adj_list[i], G.adj_list[j]);
                for (int w : W) {
                    pair<int,bool> edge = {0, false};
                    ego_network[w][i][j] = edge;
                    ego_network[w][j][i] = edge;
                }
            }
        }
    }
    //对所有节点的ego_network，计算其中每条边的support
    for(auto& ego : ego_network) {
        for(auto& node_u : ego) {
            int u = node_u.first;
            vector<int> node_u_neighbors;
            for (auto& node_v : node_u.second) {
                int v = node_v.first;
                node_u_neighbors.push_back(v);
            }
            for (auto& node_v : node_u.second) {
                int v = node_v.first;
                if (u < v) {
                    vector<int> node_v_neighbors;
                    for (auto& node_v_neighbor : ego[v]) {
                        int v_neighbor = node_v_neighbor.first;
                        node_v_neighbors.push_back(v_neighbor);
                    }
                    int support = findintersectionnum(node_u_neighbors, node_v_neighbors);
                    (node_v.second).first = support;
                    ego[v][u].first = support;
                }
            }
        }
    }
}

void GCT_INDEX::compute_truss() {
    struct Comparebysupport {
        //按support从小到大出队
        bool operator()(const tuple<int,int,int>& a, const tuple<int,int,int>& b) const {
            return get<2>(a) > get<2>(b);
        }
    };
    for(auto & ego : ego_network) {
        priority_queue<tuple<int,int,int>, vector<tuple<int,int,int>>, Comparebytruss> ego_truss_queue;
        priority_queue<tuple<int,int,int>, vector<tuple<int,int,int>>, Comparebysupport> support_queue;
        for(auto& node_u : ego) {
            int u = node_u.first;
            for (auto& node_v : node_u.second) {
                int v = node_v.first;
                if (u < v) {
                    int support = node_v.second.first;
                    support_queue.push({u, v, support});
                }
            }
        }
        while (!support_queue.empty()) {
            auto[u, v, support] = support_queue.top();
            support_queue.pop();
            if(ego[u][v].second == false) {
                ego[u][v].first += 2;
                ego[v][u].first += 2;
                ego[u][v].second = true;
                ego[v][u].second = true;
                ego_truss_queue.push({u, v, support + 2});
                vector<int> W;
                vector<int> node_u_neighbors;
                for (auto& node_u_neighbor : ego[u]) {
                    if(node_u_neighbor.second.second == false) {
                        int u_neighbor = node_u_neighbor.first;
                        node_u_neighbors.push_back(u_neighbor);
                    }
                }
                vector<int> node_v_neighbors;
                for (auto& node_v_neighbor : ego[v]) {
                    if(node_v_neighbor.second.second == false) {
                        int v_neighbor = node_v_neighbor.first;
                        node_v_neighbors.push_back(v_neighbor);
                    }
                }
                W = findintersection(node_u_neighbors, node_v_neighbors);
                for (int w : W) {
                    // 只有在support更高时才真正 -1
                    if (ego[u][w].first > support) {
                        --ego[u][w].first;
                        --ego[w][u].first;
                        support_queue.push({min(u,w), max(u,w), ego[u][w].first});
                    }
                    if (ego[v][w].first > support) {
                        --ego[v][w].first;
                        --ego[w][v].first;
                        support_queue.push({min(v,w), max(v,w), ego[v][w].first});
                    }
                }
            }
        }
        truss_queue.push_back(ego_truss_queue);
    }
}

void GCT_INDEX::build_supergraph_index() {
    for (int i = 0; i < ego_network.size(); i++) {
        auto& ego = ego_network[i];
        priority_queue<tuple<int, int, int>, vector<tuple<int, int, int>>, Comparebytruss> ego_edge_truss = truss_queue[i];

        unordered_map<int, int> super_id_map;  // node → supernode id
        unordered_map<int, pair<unordered_set<int>, int>> supernodes;  // supernode id → {nodes, truss}
        unordered_map<int, unordered_map<int, int>> super_adj_list;    // supernode graph

        // 初始化：每个点是自己的超点
        for (auto& [u, neighbors] : ego) {
            int truss_u = 0;
            for (auto& [v, info] : neighbors) {
                truss_u = max(truss_u, info.first);
            }
            super_id_map[u] = u;
            supernodes[u] = {{u}, truss_u};
        }

        // 合并 + 添加超边
        while (!ego_edge_truss.empty()) {
            auto [u, v, truss] = ego_edge_truss.top();
            ego_edge_truss.pop();
            int su = super_id_map[u], sv = super_id_map[v];
            if (su == sv) continue;

            // 判断是否已经连通
            if (isConnected(super_adj_list, su, sv)) continue;

            int truss_su = supernodes[su].second;
            int truss_sv = supernodes[sv].second;

            // 可合并
            if (truss_su == truss_sv && truss_su == truss) {
                // 合并 sv 到 su
                for (int node : supernodes[sv].first) {
                    super_id_map[node] = su;
                    supernodes[su].first.insert(node);
                }

                // 合并边
                for (auto& [nei, tval] : super_adj_list[sv]) {
                    if (nei == su) continue;
                    super_adj_list[su][nei] = tval;
                    super_adj_list[nei][su] = tval;
                    super_adj_list[nei].erase(sv);
                }

                super_adj_list.erase(sv);
                supernodes.erase(sv);
            } else {
                // 添加超边
                super_adj_list[su][sv] = truss;
                super_adj_list[sv][su] = truss;
            }
        }

        // 统计 supernodes
        for (auto& [sid, info] : supernodes) {
            int truss = info.second;
            if (truss >= node_truss_num[i].size()) node_truss_num[i].resize(truss + 1);
            node_truss_num[i][truss]++;
        }

        // 统计超边（判重）
        unordered_set<string> seen;
        for (auto& [u, neighbors] : super_adj_list) {
            for (auto& [v, truss] : neighbors) {
                if (u < v) {
                    string key = to_string(u) + "_" + to_string(v);
                    if (seen.count(key)) continue;
                    seen.insert(key);
                    if (truss >= edge_truss_num[i].size()) edge_truss_num[i].resize(truss + 1);
                    edge_truss_num[i][truss]++;
                }
            }
        }
    }
}

vector<int> GCT_INDEX::compute_diversirty(int k) {
    vector<int> result(ego_network.size(), 0);
    for (int i = 0; i < ego_network.size(); i++) {
        int count_node = 0;
        int count_edge = 0;
        int score;
        for (int j = k; j < node_truss_num[i].size(); j++) {
            count_node += node_truss_num[i][j];
        }
        for (int j = k; j < edge_truss_num[i].size(); j++) {
            count_edge += edge_truss_num[i][j];
        }
        score = count_node - count_edge;
        result[i] = score;
    }
    return result;
}

vector<int> GCT_INDEX::compute_diversirty_naive(int k) {
    vector<int> result(ego_network.size(), 0);
    for (int i = 0; i < ego_network.size(); i++) {
        auto ego = ego_network[i];  // 拷贝一份
        vector<pair<int, int>> to_remove;
        for (auto& [u, neighbors] : ego) {
            for (auto& [v, info] : neighbors) {
                if (u < v && info.first < k) {
                    to_remove.emplace_back(u, v);
                }
            }
        }
        for (auto& [u, v] : to_remove) {
            ego[u].erase(v);
            ego[v].erase(u);
            if (ego[u].empty()) ego.erase(u);
            if (ego[v].empty()) ego.erase(v);
        }
        result[i] = Num_Connected_Component(ego);
    }
    return result;
}


vector<int> compute_structure_diversity(GCT_INDEX& gctindex, int k) {
    // ############################################################################
    // # Structure Diversity Calculation
    // ############################################################################
    vector<int> diversity(gctindex.ego_network.size(), 0);
    diversity = gctindex.compute_diversirty(k);
    return diversity;
}

int main() {
    cout << "##### Loading the dataset..." << endl;

    vector<pair<int, int>> edge_list = load_edge_list("../graph.txt");
    UndirectedGraph G(edge_list);

    vector<vector<int>> ground_truths;
    ground_truths.push_back(load_ground_truth("../results(k=3).txt"));
    ground_truths.push_back(load_ground_truth("../results(k=4).txt"));
    ground_truths.push_back(load_ground_truth("../results(k=5).txt"));
    ground_truths.push_back(load_ground_truth("../results(k=6).txt"));
    vector<vector<int>> connectivity_results;

    cout << "##### Test ..." << endl;
    auto start = high_resolution_clock::now();
    GCT_INDEX gct_index;
    gct_index.build_egonetwork(G);
    gct_index.compute_truss();
    gct_index.build_supergraph_index();

    connectivity_results.push_back(compute_structure_diversity(gct_index, 3));
    connectivity_results.push_back(compute_structure_diversity(gct_index, 4));
    connectivity_results.push_back(compute_structure_diversity(gct_index, 5));
    connectivity_results.push_back(compute_structure_diversity(gct_index, 6));
    auto end = high_resolution_clock::now();
    double duration = duration_cast<milliseconds>(end - start).count() / 1000.0;

    cout << "Processing time: " << duration << " seconds" << endl;
    for (int i = 0; i < 4; ++i) {
        if (check_result(connectivity_results[i], ground_truths[i])) {
            cout << "Correct result(k=" << (i + 3) << ")" << endl;
        } else {
            cout << "Incorrect result(k=" << (i + 3) << ")" << endl;
        }
    }
    return 0;
}
