#include <iostream>
#include <vector>
#include <algorithm>
#include <cstring>
#include <map>
#include <unordered_map>
#include <queue>
using namespace std;

#define MAX_N 20
#define MAX_PATHS 1000000

int road_length[MAX_N][MAX_N]; // 道路长度矩阵
int traffic_limit[MAX_N][MAX_N]; // 交通流量限制矩阵
int N, M; // 地点数量和车辆数量

struct Vehicle{
    int start;
    int end;
};

Vehicle vehicles[MAX_N];

// 用于存储路径
struct Path{
    int nodes[MAX_N];
    int length;
    int cost;
};

Path all_paths[MAX_N][MAX_PATHS], ans_paths[MAX_N]; // 储存答案路径和每个点对的所有路径
int path_count[MAX_N]; // 每个点对的路径数量

struct Combination {
    int indices[MAX_N];
    int total_cost;

    bool operator==(const Combination &other) const {
        return memcmp(indices, other.indices, sizeof(indices)) == 0;
    }
};

struct CombinationHash {
    std::size_t operator()(const Combination& c) const {
        std::size_t res = 0;
        for(int i = 0; i < MAX_N; ++i){
            res = res * 31 + std::hash<int>()(c.indices[i]);
        }
        return res;
    }
};

struct CombinationEqual {
    bool operator()(const Combination& a, const Combination& b) const {
        return memcmp(a.indices, b.indices, sizeof(a.indices)) == 0;
    }
};

struct CompareCombination {
    bool operator()(const Combination& a, const Combination& b) const {
        return a.total_cost > b.total_cost; // 最小堆
    }
};

// 路径排序函数
int compare_paths(const void *a, const void *b) {
    Path *path_a = (Path *)a;
    Path *path_b = (Path *)b;
    return path_a->cost - path_b->cost;
}

// 深度优先搜索构建所有路径
void find_all_paths(int src, int dest, bool visited[], int current_path[], int path_length, int vehicle_index) {
    visited[src] = true;
    current_path[path_length++] = src;

    if (src == dest) {
        Path new_path;
        new_path.length = path_length;
        new_path.cost = 0;

        for (int i = 0; i < path_length; i++) {
            new_path.nodes[i] = current_path[i];
        }

        for (int i = 0; i < path_length - 1; i++) {
            new_path.cost += road_length[current_path[i]][current_path[i + 1]];
        }

        all_paths[vehicle_index][path_count[vehicle_index]++] = new_path;
    } else {
        for (int v = 0; v < N; v++) {
            if (!visited[v] && road_length[src][v] > 0) {
                find_all_paths(v, dest, visited, current_path, path_length, vehicle_index);
            }
        }
    }
    visited[src] = false;
}

// 检查路径组合是否符合流量限制（基于时间步模拟）
bool check_combination(Path *paths, int num_paths) {
    int time_usage[MAX_N][MAX_N] = {0};
    int progress[MAX_N][3];
    int book[MAX_N] = {0};
    int is_leave[MAX_N] = {0};
    int last_progress[MAX_N][2];

    for (int i = 0; i < num_paths; i++) {
        progress[i][0] = paths[i].nodes[0];
        progress[i][1] = paths[i].nodes[1];
        progress[i][2] = road_length[progress[i][0]][progress[i][1]];
    }

    while (1) {
        bool all_done = true;
        for (int i = 0; i < num_paths; i++) {
            if (is_leave[i] == 1) {
                time_usage[last_progress[i][0]][last_progress[i][1]]--;
                time_usage[last_progress[i][1]][last_progress[i][0]]--;
                is_leave[i] = 0;
                book[i] = 0;
            }
        }

        for (int i = 0; i < num_paths; i++) {
            if (progress[i][2] > 0) {
                all_done = false;
                if (book[i] == 0) {
                    time_usage[progress[i][0]][progress[i][1]]++;
                    time_usage[progress[i][1]][progress[i][0]]++;
                    book[i] = 1;
                }

                if (time_usage[progress[i][0]][progress[i][1]] > traffic_limit[progress[i][0]][progress[i][1]]) {
                    return false;
                }

                progress[i][2]--;

                if (progress[i][2] == 0) {
                    is_leave[i] = 1;
                    int current_node = progress[i][1];
                    int next_index = -1;
                    last_progress[i][0] = progress[i][0];
                    last_progress[i][1] = progress[i][1];

                    for (int j = 0; j < paths[i].length - 1; j++) {
                        if (paths[i].nodes[j] == current_node) {
                            next_index = j + 1;
                            break;
                        }
                    }

                    if (next_index != -1) {
                        progress[i][0] = current_node;
                        progress[i][1] = paths[i].nodes[next_index];
                        progress[i][2] = road_length[progress[i][0]][progress[i][1]];
                    }
                }
            }
        }

        if (all_done) break;
    }

    return true;
}

// 枚举所有合法路径组合，找到符合条件的最优组合
int find_best_combination_priority_queue(int M, Path all_paths[MAX_N][MAX_PATHS], int path_count[MAX_N], Path result_paths[MAX_N]) {
    priority_queue<Combination, vector<Combination>, CompareCombination> pq; // 优先队列，用于存储所有合法路径组合，按照总成本从小到大排序
    unordered_map<Combination, bool, CombinationHash, CombinationEqual> map; // 用于记录已经访问过的组合
    Combination initial_comb;
    memset(initial_comb.indices, 0, sizeof(initial_comb.indices));
    initial_comb.total_cost = 0;
    for (int i = 0; i < M; i++) {
        initial_comb.total_cost += all_paths[i][0].cost;
    }
    pq.push(initial_comb);
    map[initial_comb] = true;

    while (!pq.empty()) {
        Combination current = pq.top();
        pq.pop();

        Path selected_paths[MAX_N];
        for (int i = 0; i < M; i++) {
            selected_paths[i] = all_paths[i][current.indices[i]];
        }

        //cout << "Now check a new combination,the indices are: ";
        for (int i = 0; i < M; i++) {
            selected_paths[i] = all_paths[i][current.indices[i]];
            //cout << current.indices[i] << " ";
        }
        //cout << '\n';

        if (check_combination(selected_paths, M)) {
            for (int i = 0; i < M; i++) {
                result_paths[i] = selected_paths[i];
            }
            return current.total_cost;
        }
        //else
            //cout << "This combination is illegal!Come to the next one" << '\n';
        
        for (int i = 0; i < M; i++) {
            if (current.indices[i] + 1 < path_count[i]) {
                Combination next = current;
                next.indices[i]++;
                next.total_cost += all_paths[i][next.indices[i]].cost - all_paths[i][current.indices[i]].cost;
                if(!map[next]){
                    pq.push(next);
                    map[next] = true;
                }
            }
        }
    }

    return -1; // 这表示所有组合均无法满足流量限制！
}


int main() {
    cin >> N >> M;

    for (int i = 0; i < N; i++) {
        for (int j = 0; j < N; j++) {
            cin >> road_length[i][j];
        }
    }

    for (int i = 0; i < N; i++) {
        for (int j = 0; j < N; j++) {
            cin >> traffic_limit[i][j];
        }
    }

    for (int i = 0; i < M; i++) {
        cin >> vehicles[i].start >> vehicles[i].end;
        vehicles[i].start--;
        vehicles[i].end--;
    }

    for (int i = 0; i < M; i++) {
        bool visited[MAX_N] = {false};
        int current_path[MAX_N];
        path_count[i] = 0;

        find_all_paths(vehicles[i].start, vehicles[i].end, visited, current_path, 0, i);
        sort(all_paths[i], all_paths[i] + path_count[i], [](const Path &a, const Path &b) {
            return a.cost < b.cost;
        });
    }

    int best_cost = find_best_combination_priority_queue(M, all_paths, path_count, ans_paths);

    for (int i = 0; i < M; i++) {
        for (int j = 0; j < ans_paths[i].length; j++) {
            cout << ans_paths[i].nodes[j] + 1 << " ";
        }
        cout << "\n";
    }
    cout << best_cost << "\n";

    return 0;
}