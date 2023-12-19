#include <TPG.hpp>

// constructor of TPG
TPG::TPG(std::string fileName)
{
    this->numAgents = 0;
    this->numTypeTwoEdges = 0;
    // read the file
    std::ifstream file(fileName);
    std::string line;

#ifdef DEBUG
    std::cout << "Start reading the file" << std::endl;
#endif
    while (std::getline(file, line))
    {
        size_t pos = line.find(':');
        line.erase(0, pos + 2);
        std::string delimiter = "->";
        pos = 0;
        std::string token;

        Agent *agent = new Agent();
        Node *prev = NULL;
        int timeStep = 0;
        agent->robotId = getNumAgents();

        while ((pos = line.find(delimiter)) != std::string::npos)
        {
            token = line.substr(0, pos);
            token.erase(0, 1);                  // Erase the first character
            token.erase(token.length() - 1, 1); // Erase the last character

            int xCoord = std::stoi(token.substr(0, token.find(',')));
            int yCoord = std::stoi(token.substr(token.find(',') + 1));
            Node *newNode = new Node(xCoord, yCoord);
            newNode->robotId = agent->robotId;
            newNode->timeStep = timeStep;
            if (prev == NULL)
            {
                agent->Type1Next = newNode;
                agent->pathLength++;
            }
            else
            {
                prev->Type1Next = newNode;
                newNode->Type1Prev = prev;
                agent->pathLength++;
            }
            prev = newNode;
            timeStep++;

            line.erase(0, pos + delimiter.length());
        }
        addRobot(agent);
    }
#ifdef DEBUG
    std::cout << "Finish reading the file" << std::endl;
#endif
    // Add type-2 edges to TPGs
    for (auto &agent : this->agents)
    {
        int robotId = agent->robotId;
        Node *node = agent->Type1Next;
        while (node != NULL)
        {
            int currentTimeStep = node->timeStep;
            for (auto &otherAgent : this->agents)
            {
                if (otherAgent->robotId != robotId)
                {
                    Node *otherNode = otherAgent->Type1Next;
                    while (otherNode != NULL)
                    {
                        if (otherNode->coord == node->coord && otherNode->timeStep > currentTimeStep)
                        {
                            type2Edge *newType2Edge = new type2Edge();
                            newType2Edge->nodeFrom = node->Type1Next;
                            newType2Edge->nodeTo = otherNode;
                            newType2Edge->edgeId = getNumTypeTwoEdges();
                            addTypeTwoEdge(newType2Edge);
                            node->Type1Next->Type2Next.push_back(newType2Edge);
                            otherNode->Type2Prev.push_back(newType2Edge);
                        }
                        otherNode = otherNode->Type1Next;
                    }
                }
            }
            node = node->Type1Next;
        }
    }
#ifdef DEBUG
    std::cout << "Finish adding type-2 edges" << std::endl;
    std::cout << "******** TPG INFO ********" << std::endl;
    std::cout << "Number of agents: " << getNumAgents() << std::endl;
    std::cout << "Number of type-2 edges: " << getNumTypeTwoEdges() << std::endl;
#endif
}

int TPG::getNumAgents()
{
    return this->numAgents;
}

void TPG::addRobot(Agent *agent)
{
    this->numAgents++;
    this->agents.push_back(agent);
}

void TPG::addTypeTwoEdge(type2Edge *edge)
{
    this->numTypeTwoEdges++;
    this->type2Edges.push_back(edge);
}

int TPG::getNumTypeTwoEdges()
{
    return this->numTypeTwoEdges;
}

Agent *TPG::getAgent(int robotId)
{
    return this->agents[robotId];
}

type2Edge *TPG::getTypeTwoEdge(int edgeId)
{
    return this->type2Edges[edgeId];
}

void TPG::removeTypeTwoEdge(type2Edge *edge)
{
    this->type2Edges.erase(this->type2Edges.begin() + edge->edgeId);
    this->numTypeTwoEdges--;
}