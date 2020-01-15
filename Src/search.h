#ifndef SEARCH_H
#define SEARCH_H
#include "config.h"
#include "ilogger.h"
#include "searchresult.h"
#include "environmentoptions.h"
#include <list>
#include <vector>
#include <math.h>
#include <limits>
#include <chrono>
#include <set>
#include <type_traits>

double heuristic(int node_i, int node_j, int goal_i, int goal_j,
                            const EnvironmentOptions &options, 
                            const Config &config);

class Search
{
    public:
        Search();
        ~Search(void);
        SearchResult startSearch(ILogger *Logger, const Map &Map, const EnvironmentOptions &options, 
                                                                  const Config &config);

    protected:
        //CODE HERE

        //Hint 1. You definetely need class variables for OPEN and CLOSE

        //Hint 2. It's a good idea to define a heuristic calculation function, that will simply return 0
        //for non-heuristic search methods like Dijkstra

        //Hint 3. It's a good idea to define function that given a node (and other stuff needed)
        //will return it's sucessors, e.g. unordered list of nodes

        //Hint 4. working with OPEN and CLOSE is the core
        //so think of the data structures that needed to be used, about the wrap-up classes (if needed)
        //Start with very simple (and ineffective) structures like list or vector and make it work first
        //and only then begin enhancement!

        struct Comparator {
            std::string breaking_ties;
            bool operator () (const Node& node1, const Node& node2) const;
        };

        Comparator comp_nodes;
        SearchResult                    sresult; //This will store the search result
        std::list<Node>                 lppath, hppath; //
        std::pair<int, int> finish_pos;
        std::pair<int, int> start_pos;

        //CODE HERE to define other members of the class

        bool comparator(const Node& node1, const Node& node2) const;

        std::vector<Node> find_successors(const Node& curNode, 
                                          const Map &map, const EnvironmentOptions &options,
                                          const Config &config);

        void makePrimaryPath(Node curNode, Node finishNode);

        void makeSecondaryPath();

};
#endif
