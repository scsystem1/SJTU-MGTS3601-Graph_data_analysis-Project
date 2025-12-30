#include <iostream>
#include <vector>
#include <fstream>
#include <sstream>
#include <queue>
#include <algorithm>
#include <chrono>

using namespace std;

class UndirectedGraph {
public:
    int vertex_num;
    vector<vector<pair<int, int>>> adj_list;

    UndirectedGraph(const vector<tuple<int, int, int>>& edge_list) {
        vertex_num = 0;
        for (const auto& [src, dst, timestamp] : edge_list) {
            vertex_num = max({vertex_num, src + 1, dst + 1});
        }
        adj_list.resize(vertex_num);

        for (const auto& [src, dst, timestamp] : edge_list) {
            adj_list[src].emplace_back(dst, timestamp);
            adj_list[dst].emplace_back(src, timestamp);
        }
    }

};

// 双向BFS实现
bool online_bidirectional_bfs(const UndirectedGraph& G, int u, int v, int t1, int t2) {
    if (u == v) return true;

    vector<bool> visited_from_u(G.vertex_num, false);
    vector<bool> visited_from_v(G.vertex_num, false);

    queue<int> q_u, q_v;
    visited_from_u[u] = true;
    visited_from_v[v] = true;
    q_u.push(u);
    q_v.push(v);

    while (!q_u.empty() && !q_v.empty()) {
        // u进行一轮BFS
        int size_u = q_u.size();
        while (size_u--) {
            int curr = q_u.front(); q_u.pop();
            for (const auto& [neighbor, timestamp] : G.adj_list[curr]) {
                if (timestamp < t1 || timestamp > t2) continue;
                if (visited_from_u[neighbor]) continue;

                if (visited_from_v[neighbor]) return true;  // 相遇
                visited_from_u[neighbor] = true;
                q_u.push(neighbor);
            }
        }

        // v进行一轮BFS
        int size_v = q_v.size();
        while (size_v--) {
            int curr = q_v.front(); q_v.pop();
            for (const auto& [neighbor, timestamp] : G.adj_list[curr]) {
                if (timestamp < t1 || timestamp > t2) continue;
                if (visited_from_v[neighbor]) continue;

                if (visited_from_u[neighbor]) return true;  // 相遇
                visited_from_v[neighbor] = true;
                q_v.push(neighbor);
            }
        }
    }

    return false;
}

// TILL_REACH类实现
class TILL_REACH {
public:
    UndirectedGraph& G;
    int trade_off;
    vector<unsigned> ordered_vertexs; //ordered_vertexs[i]为度数第i大的节点编号
    unordered_map<unsigned, int> vertex_order; //vertex_order[i]为编号为i的节点在ordered_vertexs中的排序
    //Efficient...一文中Fig3所展示的最终存储数据结构，Lin[i].first存储所有可达节点及其时间区间在Lin[i].second中的起始位置；
    //Lin[i].second存储所有时间区间
    vector<pair<vector<pair<unsigned,unsigned>>,vector<pair<int,int>>>> Lin;

    TILL_REACH(UndirectedGraph& G, int trade_off) : G(G), trade_off(trade_off) {
        Lin.resize(G.vertex_num);
        ordered_vertexs.resize(G.vertex_num);
        for (int i = 0; i < G.vertex_num; ++i) {
            ordered_vertexs[i] = i;
        }
        sort(ordered_vertexs.begin(),ordered_vertexs.end(),
             [&](unsigned a, unsigned b) {
                 int d_a = G.adj_list[a].size();
                 int d_b = G.adj_list[b].size();
                 if(d_a != d_b) return d_a > d_b;
                 return a < b;
             });
        for (int i = 0; i < ordered_vertexs.size(); ++i) {
            vertex_order[ordered_vertexs[i]] = i;
        }
    }

    bool comp_order(unsigned ver1, unsigned ver2) {
        return vertex_order[ver1] < vertex_order[ver2];
    }

    void TILL_CONSTRUCT_STAR();
    bool SPAN_REACH(int u, int v, int t1, int t2);
};

void TILL_REACH::TILL_CONSTRUCT_STAR() {
    //三元组<v,ts,te>定义
    struct triplet {
        unsigned next_vertex_id;
        int start_time;
        int end_time;
        bool operator<(const triplet other) const {
            return(end_time- start_time)>(other.end_time - other.start_time);
        }
        triplet(unsigned vet, int ts, int te) : next_vertex_id(vet), start_time(ts), end_time(te) {}
        triplet() : next_vertex_id(0), start_time(INT_MAX), end_time(INT_MIN) {}
    };
    //原论文Algorithm3
    for(unsigned start_vertex : ordered_vertexs) {
        priority_queue<triplet> prqu;
        triplet tem_tri(start_vertex, INT_MAX, INT_MIN);
        triplet tem_tri2;
        prqu.push(tem_tri);
        while (!prqu.empty()) {
            tem_tri2 = prqu.top();
            prqu.pop();
            if(tem_tri2.next_vertex_id != start_vertex) {
                if(SPAN_REACH(start_vertex, tem_tri2.next_vertex_id, tem_tri2.start_time, tem_tri2.end_time)) continue;
                else {
                    if(Lin[tem_tri2.next_vertex_id].first.empty()) {
                        Lin[tem_tri2.next_vertex_id].first.emplace_back(start_vertex, 0);
                        Lin[tem_tri2.next_vertex_id].second.emplace_back(tem_tri2.start_time, tem_tri2.end_time);
                    }
                    else if (start_vertex == prev(Lin[tem_tri2.next_vertex_id].first.end())->first) {
                        Lin[tem_tri2.next_vertex_id].second.emplace_back(tem_tri2.start_time, tem_tri2.end_time);
                    }
                    else {
                        Lin[tem_tri2.next_vertex_id].first.emplace_back(start_vertex, Lin[tem_tri2.next_vertex_id].second.size());
                        Lin[tem_tri2.next_vertex_id].second.emplace_back(tem_tri2.start_time, tem_tri2.end_time);
                    }
                }
            }
            for (auto& [next_vertex, timestamp] : G.adj_list[tem_tri2.next_vertex_id]) {
                if(comp_order(next_vertex, start_vertex))continue;
                int ts = min(tem_tri2.start_time, timestamp);
                int te = max(tem_tri2.end_time, timestamp);
                if ((te - ts + 1) > trade_off) continue;
                else {
                    triplet tem_tri3(next_vertex, ts, te);
                    prqu.push(tem_tri3);
                }
            }
        }
    }
}

bool TILL_REACH::SPAN_REACH(int u, int v, int t1, int t2) {
    if(u == v)return true;

    if (Lin[u].first.size() == 0 && Lin[v].first.size() == 0) return false;

    //原论文中lemma9,10,根据neighbor进行快速判断
    int u_tmin = INT_MAX, u_tmax = INT_MIN, v_tmin = INT_MAX, v_tmax = INT_MIN;
    for (auto& [next_vertex, timestamp] : G.adj_list[u]) {
        u_tmin = min(u_tmin, timestamp);
        u_tmax = max(u_tmax, timestamp);
    }
    for (auto& [next_vertex, timestamp] : G.adj_list[v]) {
        v_tmin = min(v_tmin, timestamp);
        v_tmax = max(v_tmax, timestamp);
    }
    if(u_tmin > t2 || u_tmax < t1 || v_tmin > t2 || v_tmax < t1) return false;

    //分情况根据标签进行判断
    auto it1 = Lin[u].first.begin(), it2 = Lin[v].first.begin();
    while (it1 != Lin[u].first.end() || it2 != Lin[v].first.end()) {
        if(it1 == Lin[u].first.end() && it2 != Lin[v].first.end()) {
            while(it2 != Lin[v].first.end()) {
                if(it2->first == u) {
                    unsigned bg = it2->second;
                    unsigned ed;
                    if(it2 + 1 == Lin[v].first.end()) ed = Lin[v].second.size();
                    else ed = (it2 + 1)->second;
                    for (unsigned i = bg; i < ed; ++i) {
                        if (Lin[v].second[i].first >= t1 && Lin[v].second[i].second <= t2) return true;
                    }
                }
                ++it2;
            }
            return false;
        }
        else if(it2 == Lin[v].first.end() && it1 != Lin[u].first.end()) {
            while(it1 != Lin[u].first.end()) {
                if(it1->first == v) {
                    unsigned bg = it1->second;
                    unsigned ed;
                    if(it1 + 1 == Lin[u].first.end()) ed = Lin[u].second.size();
                    else ed = (it1 + 1)->second;
                    for (unsigned i = bg; i < ed; ++i) {
                        if (Lin[u].second[i].first >= t1 && Lin[u].second[i].second <= t2) return true;
                    }
                }
                ++it1;
            }
            return false;
        }
        else {
            if(it1->first == v) {
                unsigned bg = it1->second;
                unsigned ed;
                if(it1 + 1 == Lin[u].first.end()) ed = Lin[u].second.size();
                else ed = (it1 + 1)->second;
                for (unsigned i = bg; i < ed; ++i) {
                    if (Lin[u].second[i].first >= t1 && Lin[u].second[i].second <= t2) return true;
                }
            }
            else if(it2->first == u) {
                unsigned bg = it2->second;
                unsigned ed;
                if(it2 + 1 == Lin[v].first.end()) ed = Lin[v].second.size();
                else ed = (it2 + 1)->second;
                for (unsigned i = bg; i < ed; ++i) {
                    if (Lin[v].second[i].first >= t1 && Lin[v].second[i].second <= t2) return true;
                }
            }
            else if(comp_order(it1->first,it2->first))++it1;
            else if(comp_order(it2->first,it1->first))++it2;
            else {
                bool flag1 = false, flag2 = false;
                unsigned bg1 = it1->second;
                unsigned ed1;
                if(it1 + 1 == Lin[u].first.end()) ed1 = Lin[u].second.size();
                else ed1 = (it1 + 1)->second;
                for (unsigned i = bg1; i < ed1; ++i) {
                    if (Lin[u].second[i].first >= t1 && Lin[u].second[i].second <= t2){flag1 = true; break;}
                }
                unsigned bg2 = it2->second;
                unsigned ed2;
                if(it2 + 1 == Lin[v].first.end()) ed2 = Lin[v].second.size();
                else ed2 = (it2 + 1)->second;
                for (unsigned i = bg2; i < ed2; ++i) {
                    if (Lin[v].second[i].first >= t1 && Lin[v].second[i].second <= t2){flag2 = true; break;}
                }
                if(flag1 && flag2) return true;
                else {
                    ++it1;
                    ++it2;
                }
            }
        }
    }
    return false;
}

/*    else if(Lin[u].first.size() == 0 && Lin[v].first.size() != 0) {
        auto it = find_if(Lin[v].first.begin(), Lin[v].first.end(),
                [target = u](const pair<unsigned, unsigned>& p) {return p.first == target;});
        if(it == Lin[v].first.end()) return false;
        else {
            unsigned bg = it->second;
            unsigned ed;
            if(it + 1 == Lin[v].first.end()) ed = Lin[v].second.size();
            else ed = (it + 1)->second;
            for (unsigned i = bg; i < ed; ++i) {
                if (Lin[v].second[i].first >= t1 && Lin[v].second[i].second <= t2) return true;
            }
        }
        return false;
    }

    else if(Lin[u].first.size() != 0 && Lin[v].first.size() == 0) {
        auto it = find_if(Lin[u].first.begin(), Lin[u].first.end(),
                [target = v](const pair<unsigned, unsigned>& p) {return p.first == target;});
        if(it == Lin[u].first.end()) return false;
        else {
            unsigned bg = it->second;
            unsigned ed;
            if(it + 1 == Lin[u].first.end()) ed = Lin[u].second.size();
            else ed = (it + 1)->second;
            for (unsigned i = bg; i < ed; ++i) {
                if (Lin[u].second[i].first >= t1 && Lin[u].second[i].second <= t2) return true;
            }
        }
        return false;
    }

    else {
        auto it1 = Lin[u].first.begin(), it2 = Lin[v].first.begin();
        while (it1 != Lin[u].first.end() || it2 != Lin[v].first.end()) {
            if(it1->first == v) {
                unsigned bg = it1->second;
                unsigned ed;
                if(it1 + 1 == Lin[u].first.end()) ed = Lin[u].second.size();
                else ed = (it1 + 1)->second;
                for (unsigned i = bg; i < ed; ++i) {
                    if (Lin[u].second[i].first >= t1 && Lin[u].second[i].second <= t2) return true;
                }
            }
            else if(it2->first == u) {
                unsigned bg = it2->second;
                unsigned ed;
                if(it2 + 1 == Lin[v].first.end()) ed = Lin[v].second.size();
                else ed = (it2 + 1)->second;
                for (unsigned i = bg; i < ed; ++i) {
                    if (Lin[v].second[i].first >= t1 && Lin[v].second[i].second <= t2) return true;
                }
            }
            else if(comp_order(it1->first,it2->first))++it1;
            else if(comp_order(it2->first,it1->first))++it2;
            else {
                bool flag1 = false, flag2 = false;
                unsigned bg1 = it1->second;
                unsigned ed1;
                if(it1 + 1 == Lin[u].first.end()) ed1 = Lin[u].second.size();
                else ed1 = (it1 + 1)->second;
                for (unsigned i = bg1; i < ed1; ++i) {
                    if (Lin[u].second[i].first >= t1 && Lin[u].second[i].second <= t2){flag1 = true; break;}
                }
                unsigned bg2 = it2->second;
                unsigned ed2;
                if(it2 + 1 == Lin[v].first.end()) ed2 = Lin[v].second.size();
                else ed2 = (it2 + 1)->second;
                for (unsigned i = bg2; i < ed2; ++i) {
                    if (Lin[v].second[i].first >= t1 && Lin[v].second[i].second <= t2){flag2 = true; break;}
                }
                if(flag1 && flag2) return true;
                else {
                    ++it1;
                    ++it2;
                }
            }
        }
        return false;
    }
}
*/


vector<int> HistoricalConnectivity(UndirectedGraph& G, const vector<tuple<int, int, int, int>>& queries) {
    vector<int> result;
    //TILL_REACH tr(G, 10);
    //tr.TILL_CONSTRUCT_STAR();
    for (const auto& [u, v, ts, te] : queries) {

    // ############################################################################
    // # TODO: Your code here~
    // ############################################################################
        //bool span_reach = tr.SPAN_REACH(u, v, ts, te);
        //result.push_back(span_reach ? 1 : 0);
        int res = online_bidirectional_bfs(G,u,v,ts,te) ? 1 : 0;
        result.push_back(res);
    // ############################################################################

    }
    return result;
}

vector<tuple<int, int, int>> loadEdges(const string& filename) {
    ifstream file(filename);
    vector<tuple<int, int, int>> edges;
    int u, v, t;

    while (file >> u >> v >> t) {
        edges.emplace_back(u, v, t);
    }
    return edges;
}

vector<tuple<int, int, int, int>> loadQueries(const string& filename) {
    ifstream file(filename);
    vector<tuple<int, int, int, int>> queries;
    int u, v, ts, te;

    while (file >> u >> v >> ts >> te) {
        queries.emplace_back(u, v, ts, te);
    }
    return queries;
}

vector<int> loadResults(const string& filename) {
    ifstream file(filename);
    vector<int> results;
    int val;

    while (file >> val) {
        results.push_back(val);
    }
    return results;
}

bool checkResult(const vector<int>& prediction, const vector<int>& ground_truth) {
    return prediction == ground_truth;
}

int main() {
    cout << "\n##### Loading the dataset..." << endl;
    vector<tuple<int, int, int>> edge_list = loadEdges("../graph2.txt");
    vector<tuple<int, int, int, int>> queries = loadQueries("../queries2.txt");
    vector<int> ground_truth = loadResults("../results2.txt");

    UndirectedGraph G(edge_list);

    cout << "\n##### Test ..." << endl;
    auto start = chrono::high_resolution_clock::now();
    vector<int> connectivity_results = HistoricalConnectivity(G, queries);
    auto end = chrono::high_resolution_clock::now();
    chrono::duration<double> elapsed = end - start;

    cout << "Processing time: " << elapsed.count() << " seconds" << endl;
    cout << (checkResult(connectivity_results, ground_truth) ? "Correct result" : "Incorrect result") << endl;


    cout << "Query count: " << queries.size() << endl;
    cout << "Ground truth size: " << ground_truth.size() << endl;
    cout << "Your output size: " << connectivity_results.size() << endl;

    for (int i = 0; i < connectivity_results.size(); ++i) {
        cout << "Q" << i << ": pred=" << connectivity_results[i] << ", truth=" << ground_truth[i] << endl;
    }
    return 0;
}
