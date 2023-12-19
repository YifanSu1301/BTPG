#include "util.hpp"

class TPG
{
private:
    int numAgents;
    int numTypeTwoEdges;
    std::vector<Agent *> agents;
    std::vector<type2Edge *> type2Edges;

public:
    TPG(std::string fileName);
    // ~TPG();

    int getNumAgents();
    int getNumTypeTwoEdges();
    void addRobot(Agent *agent);
    void addTypeTwoEdge(type2Edge *edge);
    void removeTypeTwoEdge(type2Edge *edge);

    Agent *getAgent(int robotId);
    type2Edge *getTypeTwoEdge(int edgeId);
};