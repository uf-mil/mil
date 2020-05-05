#include <nanoflann.hpp>
#include <Eigen/Core>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graphviz.hpp>
#include <iostream>
#include <fstream>
#include <random>


class RRT
{
public:
    using StateT = Eigen::Vector2d;
    RRT();
    void populate();
    void visualize();
    void SetGoal(StateT _goal);
    void SetOrigin(StateT _origin);
    /// Set probability that RRT will pick a random value rather than moving towards goal
    void SetExplorationBias(double _bias); 
    void explore();
private:
    using NodesContainerT = std::vector<StateT>;
    StateT goal_;
    StateT origin_;
    NodesContainerT nodes_;
    boost::adjacency_list<boost::vecS, boost::vecS, boost::directedS> graph_;
    std::default_random_engine rng_;
    /// Bernoulli (boolean) distribution where positive value means explore (pick a random value)
    std::bernoulli_distribution exploration_bias_;
};

RRT::RRT()
{
};

void RRT::SetGoal(StateT _goal)
{
    goal_ = _goal;
}

void RRT::SetOrigin(StateT _origin)
{
    origin_ = _origin;
}

void RRT::SetExplorationBias(double _bias)
{
    exploration_bias_ = std::bernoulli_distribution(_bias);
}


void RRT::explore()
{
  if (exploration_bias_(rng_)) {
    std::cout << "Explore" << std::endl;
  } else {
    std::cout << "Not Explore" << std::endl;
  }
}

void RRT::populate()
{
    nodes_.push_back(StateT(0, 1));
    nodes_.push_back(StateT(1, 1));
    boost::add_edge(0, 1, graph_);
}

void RRT::visualize()
{
    std::ofstream file;
    file.open("graph.dot");
    boost::write_graphviz(file, graph_);
}

int main()
{
    RRT r;
    r.SetExplorationBias(0.9);
    for (size_t i = 0; i < 100; ++i) r.explore();
    r.populate();
    r.visualize();
}
