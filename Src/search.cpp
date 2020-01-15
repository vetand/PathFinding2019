#include "search.h"

Search::Search()
{
//set defaults here
}

Search::~Search() {}

double heuristic(int node_i, int node_j, 
                         int goal_i, int goal_j,
                         const EnvironmentOptions &options, 
                         const Config &config) {
    if (config.SearchParams[CN_SP_ST] == CN_SP_ST_DIJK) {
        return 0;
    } else if (config.SearchParams[CN_SP_ST] == CN_SP_ST_ASTAR) {
        int di = std::abs(node_i - goal_i);
        int dj = std::abs(node_j - goal_j);
        if (options.metrictype == CN_SP_MT_DIAG) {
            return (double)std::min(di, dj) * CN_SQRT_TWO + (double)std::max(di, dj) - 
                                                            (double)std::min(di, dj);
        } else if (options.metrictype == CN_SP_MT_MANH) {
            return di + dj;
        } else if (options.metrictype == CN_SP_MT_EUCL) {
            return sqrt(di * di + dj * dj);
        } else if (options.metrictype == CN_SP_MT_CHEB) {
            return std::max(di, dj);
        }
    }
    return 0;
}

bool Search::Comparator::operator () (const Node& node1, const Node& node2) const {
    if (this->breaking_ties == "g-min") {
        return std::tie(node1.F, node1.g, node1) < std::tie(node2.F, node2.g, node2);
    } else {
        return std::tie(node1.F, node1.H, node1) < std::tie(node2.F, node2.H, node2);
    }
}

std::vector<Node> Search::find_successors(const Node& curNode, 
                                          const Map &map, const EnvironmentOptions &options,
                                          const Config &config) {
    std::vector<Node> answer;
    for (int dx = -1; dx <= 1; ++dx) {
        for (int dy = -1; dy <= 1; ++dy) {
            if (dx == 0 && dy == 0) {
                continue;
            }
            int new_i = curNode.i + dx;
            int new_j = curNode.j + dy;
            if (!map.CellOnGrid(new_i, new_j) || map.CellIsObstacle(new_i, new_j)) {
                continue;
            }
            Node scNode = Node(new_i, new_j);
            if (std::abs(dx) == 1 && std::abs(dy) == 1) {
                if (!options.allowdiagonal) {
                    continue;
                }
                int cnt_near = 0;
                if (map.CellIsObstacle(new_i, curNode.j)) {
                    ++cnt_near;
                }
                if (map.CellIsObstacle(curNode.i, new_j)) {
                    ++cnt_near;
                }
                if ((cnt_near >= 1 && !options.cutcorners) || 
                    (cnt_near >= 2 && !options.allowsqueeze)) {
                    continue;
                }
                scNode.g = curNode.g + CN_SQRT_TWO;
            } else {
                scNode.g = curNode.g + 1;
            }
            scNode.H = heuristic(scNode.i, scNode.j, this->finish_pos.first, 
                            this->finish_pos.second, options, config) * config.SearchParams[CN_SP_HW];
            scNode.F = scNode.H + scNode.g;
            answer.push_back(scNode);
        }
    }
    return answer;
}

SearchResult Search::startSearch(ILogger *Logger, const Map &map, 
                                 const EnvironmentOptions &options, const Config &config) {
    //need to implement
    auto time_start = clock();

    if (config.SearchParams[CN_SP_ST] == CN_SP_ST_DIJK || 
        (config.SearchParams[CN_SP_ST] == CN_SP_ST_ASTAR && 
            config.SearchParams[CN_SP_BT] == CN_SP_BT_GMIN)) {
        this->comp_nodes.breaking_ties = "g-min";
    } else {
        this->comp_nodes.breaking_ties = "g-max";
    }
    sresult.pathfound = false;
    sresult.numberofsteps = 0;
    sresult.nodescreated = 0;
    sresult.pathlength = 0;
    this->start_pos = std::make_pair(map.start_i, map.start_j);
    this->finish_pos = std::make_pair(map.goal_i, map.goal_j);

    Node startNode = Node(this->start_pos.first, this->start_pos.second);
    startNode.g = 0;
    startNode.H = heuristic(startNode.i, startNode.j, this->finish_pos.first, 
                            this->finish_pos.second, options, config);
    startNode.F = startNode.H * config.SearchParams[CN_SP_HW];
    std::set<Node, Comparator> OPEN;
    std::set<Node> CLOSED;
    OPEN.insert(startNode);
    Node finish_node(0, 0);

    while (OPEN.size() > 0) {
        Node curNode = *OPEN.begin();
        OPEN.erase(OPEN.begin());
        if (CLOSED.find(curNode) != CLOSED.end()) {
            continue;
        }
        CLOSED.insert(curNode);
        if (curNode.i == this->finish_pos.first && curNode.j == this->finish_pos.second) {
            finish_node = curNode;
            sresult.pathfound = true;
            break;
        }
        std::vector<Node> successors = this->find_successors(curNode, map, options, config);
        for (auto& node : successors) {
            if (CLOSED.find(node) == CLOSED.end()) {
                node.parent = (Node*)&(*CLOSED.find(curNode));
                OPEN.insert(node);
            }
        }
        ++sresult.numberofsteps;
    }

    for (auto& node : OPEN) {
        if (CLOSED.find(node) == CLOSED.end()) {
            CLOSED.insert(node);
        }
    }

    sresult.nodescreated = CLOSED.size();

    if (sresult.pathfound) {
        this->makePrimaryPath(finish_node, startNode);
        this->makeSecondaryPath();
        sresult.pathlength = finish_node.g;
        sresult.hppath = &hppath;
        sresult.lppath = &lppath;
    }
    sresult.time = (double)(clock() - time_start) / CLOCKS_PER_SEC;

    /*sresult.pathfound = ;
    sresult.nodescreated =  ;
    sresult.numberofsteps = ;
    sresult.time = ;
    sresult.hppath = &hppath; //Here is a constant pointer
    sresult.lppath = &lppath;*/
    return sresult;
}

void Search::makePrimaryPath(Node curNode, Node startNode) {
    //need to implement
    this->lppath.clear();
    while (curNode.parent != nullptr) {
        this->lppath.insert(this->lppath.begin(), curNode);
        curNode = *curNode.parent;
    }
    this->lppath.insert(this->lppath.begin(), startNode);
}

void Search::makeSecondaryPath() {
    //need to implement
    this->hppath.clear();
    std::pair<int, int> current_direction = {0, 0};
    for (auto it = this->lppath.begin(); it != this->lppath.end(); ++it) {
        auto next_it = it;
        ++next_it;
        if (next_it == this->lppath.end()) {
            this->hppath.insert(this->hppath.end(), *it);
            break;
        }
        std::pair<int, int> new_direction = {next_it->i - it->i, next_it->j - it->j};
        if (current_direction != current_direction) {
            this->hppath.insert(this->hppath.end(), *it);
        }
        current_direction = new_direction;
    }
    this->hppath.push_back(this->lppath.back());
}
