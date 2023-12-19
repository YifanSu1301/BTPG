#include "Sim.hpp"
#include <algorithm>
/**
 * @brief Constructor for Sim class
 */
Sim::Sim(int seed, int numRobots)
{
    this->seed = seed;
    srand(seed);
    // Choose delayed robots in this simulation
    double numDelayedRobots = 0.1 * numRobots;
    for (double i = 0; i < numDelayedRobots; ++i)
    {
        int random_number = rand() % numRobots;
        this->DelayedRobots.push_back(random_number);
    }
}

/**
 * @brief Simulation of BTPG
 */
int Sim::Simulate(BTPG *btpg_)
{
    this->mode = 1;
    this->btpg = btpg_;
    srand(this->seed);
    // 1a. Initialize generated Path
    for (int i = 0; i < this->btpg->getNumAgents(); ++i)
    {
        std::vector<Coord> path;
        path.push_back(this->btpg->getAgent(i)->Type1Next->coord);
        this->BTPGGeneratedPath.push_back(path);
    }

    // 1b.Initialize visited nodes
    std::vector<std::vector<bool>> visited;
    for (int i = 0; i < this->btpg->getNumAgents(); ++i)
    {
        std::vector<bool> visitedAgent;
        for (int j = 0; j < this->btpg->getAgent(i)->pathLength; ++j)
        {
            if (j == 0)
                visitedAgent.push_back(true);
            else
                visitedAgent.push_back(false);
        }
        visited.push_back(visitedAgent);
    }

    // 1c. Initialize finished Agent list
    std::vector<bool> finishedAgent;
    for (int i = 0; i < this->btpg->getNumAgents(); ++i)
    {
        finishedAgent.push_back(false);
    }

    // 1d. Initialize robot stop numbers
    std::unordered_map<int, int> robotStopNumbers;
    for (int i = 0; i < this->DelayedRobots.size(); ++i)
    {
        robotStopNumbers[this->DelayedRobots[i]] = 0;
    }

// 2. Start simulation
#ifdef DEBUG
    std::cout << "Start BTPG simulation" << std::endl;
#endif
    this->BTPGTotalTimeStep = 0;
    while (std::find(finishedAgent.begin(), finishedAgent.end(), false) != finishedAgent.end())
    {
        this->BTPGTotalTimeStep++;
        // 2a.Decide which agent can move at this timestep
        std::vector<int> movableAgents;
        DecideMovableAgents(movableAgents, robotStopNumbers);
        // print out all the movable agents
        SimulateTimeStep(movableAgents, visited, finishedAgent);
    }
#ifdef DEBUG
    std::cout << "Finish BTPG simulation" << std::endl;
#endif
    CheckBTPGPathValidity();
#ifdef DEBUG
    std::cout << "Finish checking path validity" << std::endl;
#endif
    GetStatistics();

    return 1;
}

/**
 * @brief Simulation of TPG
 */
int Sim::Simulate(TPG *tpg_)
{
    this->mode = 0;
    this->tpg = tpg_;
    srand(this->seed);
    // 1a. Initialize generated Path
    for (int i = 0; i < this->tpg->getNumAgents(); ++i)
    {
        std::vector<Coord> path;
        path.push_back(this->tpg->getAgent(i)->Type1Next->coord);
        this->TPGGeneratedPath.push_back(path);
    }

    // 1b.Initialize visited nodes
    std::vector<std::vector<bool>> visited;
    for (int i = 0; i < this->tpg->getNumAgents(); ++i)
    {
        std::vector<bool> visitedAgent;
        for (int j = 0; j < this->tpg->getAgent(i)->pathLength; ++j)
        {
            if (j == 0)
                visitedAgent.push_back(true);
            else
                visitedAgent.push_back(false);
        }
        visited.push_back(visitedAgent);
    }

    // 1c. Initialize finished Agent list
    std::vector<bool> finishedAgent;
    for (int i = 0; i < this->tpg->getNumAgents(); ++i)
    {
        finishedAgent.push_back(false);
    }

    // 1d. Initialize robot stop numbers
    std::unordered_map<int, int> robotStopNumbers;
    for (int i = 0; i < this->DelayedRobots.size(); ++i)
    {
        robotStopNumbers[this->DelayedRobots[i]] = 0;
    }

// 2. Start simulation
#ifdef DEBUG
    std::cout << "Start TPG simulation" << std::endl;
#endif
    this->TPGTotalTimeStep = 0;
    while (std::find(finishedAgent.begin(), finishedAgent.end(), false) != finishedAgent.end())
    {
        this->TPGTotalTimeStep++;
        // 2a.Decide which agent can move at this timestep
        std::vector<int> movableAgents;
        DecideTPGMovableAgents(movableAgents, robotStopNumbers);
        // print out all the movable agents
        SimulateTPGTimeStep(movableAgents, visited, finishedAgent);
    }
#ifdef DEBUG
    std::cout << "Finish TPG simulation" << std::endl;
#endif
    CheckTPGPathValidity();
#ifdef DEBUG
    std::cout << "Finish checking path validity" << std::endl;
#endif
    GetTPGStatistics();

    return 1;
}

/**
 * @brief Simulation of no delay TPG
 */
void Sim::SimulateNoDelay(TPG *tpg_)
{

    this->mode = 0;
    this->tpg = tpg_;
    srand(this->seed);
    // 1a. Initialize generated Path
    for (int i = 0; i < this->tpg->getNumAgents(); ++i)
    {
        std::vector<Coord> path;
        path.push_back(this->tpg->getAgent(i)->Type1Next->coord);
        this->TPGGeneratedPathNoDelay.push_back(path);
    }

    // 1b.Initialize visited nodes
    std::vector<std::vector<bool>> visited;
    for (int i = 0; i < this->tpg->getNumAgents(); ++i)
    {
        std::vector<bool> visitedAgent;
        for (int j = 0; j < this->tpg->getAgent(i)->pathLength; ++j)
        {
            if (j == 0)
                visitedAgent.push_back(true);
            else
                visitedAgent.push_back(false);
        }
        visited.push_back(visitedAgent);
    }

    // 1c. Initialize finished Agent list
    std::vector<bool> finishedAgent;
    for (int i = 0; i < this->tpg->getNumAgents(); ++i)
    {
        finishedAgent.push_back(false);
    }

// 2. Start simulation
#ifdef DEBUG
    std::cout << "Start TPGWoDelay simulation" << std::endl;
#endif
    this->TPGTotalTimeStepNoDelay = 0;
    while (std::find(finishedAgent.begin(), finishedAgent.end(), false) != finishedAgent.end())
    {
        this->TPGTotalTimeStepNoDelay++;
        // 2a.Decide which agent can move at this timestep
        std::vector<int> movableAgents;
        for (int i = 0; i < this->tpg->getNumAgents(); ++i)
        {
            movableAgents.push_back(i);
        }
        // print out all the movable agents
        SimulateTPGWoDelayTimeStep(movableAgents, visited, finishedAgent);
    }
#ifdef DEBUG
    std::cout << "Finish TPGWoDelay simulation" << std::endl;
#endif
    CheckTPGWoDelayPathValidity();
#ifdef DEBUG
    std::cout << "Finish checking path validity" << std::endl;
#endif
    GetTPGWoDelayStatistics();

    return;
}

/******************************************/
/************ Helper Functions ***********/
/******************************************/

/**
 * @brief Get the Statistics object
 *
 */

void Sim::GetStatistics()
{
    std::cout << "********* BTPG Statistics *********" << std::endl;
    std::cout << "BTPG Total Time Step: " << this->BTPGTotalTimeStep << std::endl;
    int averageTime = 0;
    for (int i = 0; i < this->btpg->getNumAgents(); ++i)
    {
        averageTime += this->btpg->getAgent(i)->BTPGFinishedTime;
    }
    averageTime /= this->btpg->getNumAgents();
    this->BTPGaverageTime = averageTime;
    std::cout << "BTPG Average Time Step: " << averageTime << std::endl;
    std::cout << "********* ************** *********" << std::endl;
}
void Sim::GetTPGStatistics()
{
    std::cout << "********* TPG Statistics *********" << std::endl;
    std::cout << "TPG Total Time Step: " << this->TPGTotalTimeStep << std::endl;
    int averageTime = 0;
    for (int i = 0; i < this->tpg->getNumAgents(); ++i)
    {
        averageTime += this->tpg->getAgent(i)->TPGFinishedTime;
    }
    averageTime /= this->tpg->getNumAgents();
    this->TPGaverageTime = averageTime;
    std::cout << "TPG Average Time Step: " << averageTime << std::endl;
    std::cout << "********* ************** *********" << std::endl;
}

void Sim::GetTPGWoDelayStatistics()
{
    std::cout << "********* TPG Wo Delay Statistics *********" << std::endl;
    std::cout << "TPG Wo Delay Total Time Step: " << this->TPGTotalTimeStepNoDelay << std::endl;
    int totalTime = 0;
    for (int i = 0; i < this->tpg->getNumAgents(); ++i)
    {
        totalTime += this->tpg->getAgent(i)->TPGFinishedTimeNoDelay;
    }
    int averageTime = totalTime / this->tpg->getNumAgents();
    std::cout << "TPG Wo Delay Average Time Step: " << averageTime << std::endl;
    // int expectedDelay = 0.1 * ((1.0 / (1.0 - 0.3) - 1) * (5 + 1) + 1) * averageTime + averageTime * (1.0 - 0.1);
    int expectedDelay = (totalTime + this->totalDelay)/this->tpg->getNumAgents();
    this->expectedDelay = expectedDelay;
    std::cout << "Expected Delay: " << expectedDelay << std::endl;
    std::cout << "********* ************** *********" << std::endl;
}

void Sim::CheckTPGWoDelayPathValidity()
{

    // check if the starting point and end points are right
    for (int i = 0; i < this->TPGGeneratedPathNoDelay.size(); i++)
    {
        // find the start and end coord from TPG
        Node *node = this->tpg->getAgent(i)->Type1Next;
        while (node->Type1Next != NULL)
        {
            node = node->Type1Next;
        }
        Coord endCoord = node->coord;
        Coord startCoord = this->tpg->getAgent(i)->Type1Next->coord;
        if (this->TPGGeneratedPathNoDelay[i][0] != startCoord || this->TPGGeneratedPathNoDelay[i][this->TPGGeneratedPathNoDelay[i].size() - 1] != endCoord)
        {
            std::cout << "Path Invalid - wrong start and wrong coord" << std::endl;
            exit(1);
        }
    }

    // check if there is a collision
    for (int i = 0; i < this->TPGGeneratedPathNoDelay.size(); i++)
    {
        for (int j = i + 1; j < this->TPGGeneratedPathNoDelay.size(); j++)
        {
            for (int k = 0; k < this->TPGGeneratedPathNoDelay[i].size(); k++)
            {

                if (k < this->TPGGeneratedPathNoDelay[j].size() && this->TPGGeneratedPathNoDelay[i][k] == this->TPGGeneratedPathNoDelay[j][k])
                {
                    std::cout << "Path Invalid - Collision - "
                              << "robot: " << i << " and robot: " << j << " at timestep " << k << std::endl;
                    std::cout << this->TPGGeneratedPathNoDelay[j].size() << " | " << this->TPGGeneratedPathNoDelay[i].size() << std::endl;
                    exit(1);
                }
            }
        }
    }

    // check only one move without jump
    for (int i = 0; i < this->TPGGeneratedPathNoDelay.size(); i++)
    {
        for (int j = 0; j < this->TPGGeneratedPathNoDelay[i].size() - 1; j++)
        {
            if (abs(this->TPGGeneratedPathNoDelay[i][j].x - this->TPGGeneratedPathNoDelay[i][j + 1].x) + abs(this->TPGGeneratedPathNoDelay[i][j].y - this->TPGGeneratedPathNoDelay[i][j + 1].y) > 1)
            {
                std::cout << "Path Invalid - Jump " << std::endl;
                exit(1);
            }
        }
    }
    return;
}

void Sim::CheckTPGPathValidity()
{

    // check if the starting point and end points are right
    for (int i = 0; i < TPGGeneratedPath.size(); i++)
    {
        // find the start and end coord from TPG
        Node *node = this->tpg->getAgent(i)->Type1Next;
        while (node->Type1Next != NULL)
        {
            node = node->Type1Next;
        }
        Coord endCoord = node->coord;
        Coord startCoord = this->tpg->getAgent(i)->Type1Next->coord;
        if (TPGGeneratedPath[i][0] != startCoord || TPGGeneratedPath[i][TPGGeneratedPath[i].size() - 1] != endCoord)
        {
            std::cout << "Path Invalid - wrong start and wrong coord" << std::endl;
            exit(1);
        }
    }

    // check if there is a collision
    for (int i = 0; i < TPGGeneratedPath.size(); i++)
    {
        for (int j = i + 1; j < TPGGeneratedPath.size(); j++)
        {
            for (int k = 0; k < TPGGeneratedPath[i].size(); k++)
            {

                if (k < TPGGeneratedPath[j].size() && TPGGeneratedPath[i][k] == TPGGeneratedPath[j][k])
                {
                    std::cout << "Path Invalid - Collision - "
                              << "robot: " << i << " and robot: " << j << " at timestep " << k << std::endl;
                    std::cout << TPGGeneratedPath[j].size() << " | " << TPGGeneratedPath[i].size() << std::endl;
                    exit(1);
                }
            }
        }
    }

    // check only one move without jump
    for (int i = 0; i < TPGGeneratedPath.size(); i++)
    {
        for (int j = 0; j < TPGGeneratedPath[i].size() - 1; j++)
        {
            if (abs(TPGGeneratedPath[i][j].x - TPGGeneratedPath[i][j + 1].x) + abs(TPGGeneratedPath[i][j].y - TPGGeneratedPath[i][j + 1].y) > 1)
            {
                std::cout << "Path Invalid - Jump " << std::endl;
                std::cout << TPGGeneratedPath[i][j].x << " " << TPGGeneratedPath[i][j].y << " " << TPGGeneratedPath[i][j + 1].x << " " << TPGGeneratedPath[i][j + 1].y << std::endl;
                std::cout << TPGGeneratedPath[i].size() << std::endl;
                exit(1);
            }
        }
    }
    return;
}

void Sim::CheckBTPGPathValidity()
{

    // check if the starting point and end points are right
    for (int i = 0; i < BTPGGeneratedPath.size(); i++)
    {
        // find the start and end coord from TPG
        Node *node = this->btpg->getAgent(i)->Type1Next;
        while (node->Type1Next != NULL)
        {
            node = node->Type1Next;
        }
        Coord endCoord = node->coord;
        Coord startCoord = this->btpg->getAgent(i)->Type1Next->coord;
        if (BTPGGeneratedPath[i][0] != startCoord || BTPGGeneratedPath[i][BTPGGeneratedPath[i].size() - 1] != endCoord)
        {
            std::cout << "Path Invalid - wrong start and wrong coord" << std::endl;
            exit(1);
        }
    }

    // check if there is a collision
    for (int i = 0; i < BTPGGeneratedPath.size(); i++)
    {
        for (int j = i + 1; j < BTPGGeneratedPath.size(); j++)
        {
            for (int k = 0; k < BTPGGeneratedPath[i].size(); k++)
            {

                if (k < BTPGGeneratedPath[j].size() && BTPGGeneratedPath[i][k] == BTPGGeneratedPath[j][k])
                {
                    std::cout << "Path Invalid - Collision - "
                              << "robot: " << i << " and robot: " << j << " at timestep " << k << std::endl;
                    std::cout << BTPGGeneratedPath[j].size() << " | " << BTPGGeneratedPath[i].size() << std::endl;
                    exit(1);
                }
            }
        }
    }

    // check only one move without jump
    for (int i = 0; i < BTPGGeneratedPath.size(); i++)
    {
        for (int j = 0; j < BTPGGeneratedPath[i].size() - 1; j++)
        {
            if (abs(BTPGGeneratedPath[i][j].x - BTPGGeneratedPath[i][j + 1].x) + abs(BTPGGeneratedPath[i][j].y - BTPGGeneratedPath[i][j + 1].y) > 1)
            {
                std::cout << "Path Invalid - Jump " << std::endl;
                exit(1);
            }
        }
    }
    return;
}
void Sim::SimulateTimeStep(std::vector<int> &movableAgents, std::vector<std::vector<bool>> &visited, std::vector<bool> &finishedAgent)
{

    // 1. Check which agent can move based on BTPG
    int NumRobotsCanMoveBasedOnBTPG = 0;
    std::vector<int> CheckMoveAgents;
    for (int i = 0; i < this->btpg->getNumAgents(); i++)
    {
        CheckMoveAgents.push_back(i);
    }
    std::vector<int> tempCheckMoveAgents;
    std::vector<std::pair<std::pair<int, int>, std::pair<int, int>>> BTPGrotationStopRobots;
    int newVisit = 1;
    while (newVisit != 0)
    {
        newVisit = 0;
        BTPGrotationStopRobots.clear();
        for (auto i : CheckMoveAgents)
        {
            if (finishedAgent[i])
                continue;
            int BTPGNextIdx = std::find(visited[i].begin(), visited[i].end(), false) - visited[i].begin();
            Node *BTPGNextNode = this->btpg->getAgent(i)->Type1Next;
            for (int j = 0; j < BTPGNextIdx; ++j)
            {
                BTPGNextNode = BTPGNextNode->Type1Next;
            }
            bool CanVisit = true;
            std::vector<int> CheckBiPair;
            for (auto edge : BTPGNextNode->Type2Prev)
            {
                if (this->mode == 1 && edge->isBidirectional)
                {
                    if (!this->btpg->getBiPair(edge->biPairId)->isVisited)
                    {
                        CheckBiPair.push_back(edge->biPairId);
                        continue;
                    }
                }
                if (visited[edge->nodeFrom->robotId][edge->nodeFrom->timeStep] == false)
                {
                    CanVisit = false;
                    BTPGrotationStopRobots.push_back(std::make_pair(std::make_pair(i, BTPGNextIdx), std::make_pair(edge->nodeFrom->robotId, edge->nodeFrom->timeStep)));
                    tempCheckMoveAgents.push_back(i);
                    break;
                }
            }
            if (CanVisit)
            {
                NumRobotsCanMoveBasedOnBTPG++;

                if (std::find(movableAgents.begin(), movableAgents.end(), i) != movableAgents.end())
                {
                    newVisit++;
                    visited[i][BTPGNextIdx] = true;
                    if (BTPGNextIdx == this->btpg->getAgent(i)->pathLength - 1)
                    {
                        finishedAgent[i] = true;
                        this->btpg->getAgent(i)->BTPGFinishedTime = this->BTPGTotalTimeStep;
                    }
                    this->BTPGGeneratedPath[i].push_back(BTPGNextNode->coord);
                    for (auto biPairId : CheckBiPair)
                    {
                        if (this->btpg->getTypeTwoEdge(this->btpg->getBiPair(biPairId)->flippedId)->nodeFrom->robotId == i)
                        {
                            this->numBidirectionalEdgesIsUsed++;
                        }
                        this->btpg->getBiPair(biPairId)->isVisited = true;
                    }
                }

            }
        }
        // 2. Rotations
        newVisit += moveRotation(BTPGrotationStopRobots, movableAgents, visited, finishedAgent, NumRobotsCanMoveBasedOnBTPG, tempCheckMoveAgents);

        CheckMoveAgents = tempCheckMoveAgents;
        tempCheckMoveAgents.clear();
    }

    if (NumRobotsCanMoveBasedOnBTPG == 0)
    {
        std::cout << "Deadlock: No robot can move based on BTPG" << std::endl;
        exit(1);
    }

    return;
}

void Sim::SimulateTPGTimeStep(std::vector<int> &movableAgents, std::vector<std::vector<bool>> &visited, std::vector<bool> &finishedAgent)
{

    // 1. Check which agent can move based on TPG
    int NumRobotsCanMoveBasedOnTPG = 0;
    std::vector<int> CheckMoveAgents;
    for (int i = 0; i < this->tpg->getNumAgents(); i++)
    {
        CheckMoveAgents.push_back(i);
    }
    std::vector<int> tempCheckMoveAgents;
    std::vector<std::pair<std::pair<int, int>, std::pair<int, int>>> TPGrotationStopRobots;
    int newVisit = 1;
    while (newVisit != 0)
    {
        newVisit = 0;
        TPGrotationStopRobots.clear();
        for (auto i : CheckMoveAgents)
        {
            if (finishedAgent[i])
                continue;
            int TPGNextIdx = std::find(visited[i].begin(), visited[i].end(), false) - visited[i].begin();
            Node *TPGNextNode = this->tpg->getAgent(i)->Type1Next;
            for (int j = 0; j < TPGNextIdx; ++j)
            {
                TPGNextNode = TPGNextNode->Type1Next;
            }
            bool CanVisit = true;

            for (auto edge : TPGNextNode->Type2Prev)
            {

                if (visited[edge->nodeFrom->robotId][edge->nodeFrom->timeStep] == false)
                {
                    CanVisit = false;
                    TPGrotationStopRobots.push_back(std::make_pair(std::make_pair(i, TPGNextIdx), std::make_pair(edge->nodeFrom->robotId, edge->nodeFrom->timeStep)));
                    tempCheckMoveAgents.push_back(i);
                    break;
                }
            }
            if (CanVisit)
            {
                NumRobotsCanMoveBasedOnTPG++;

                if (std::find(movableAgents.begin(), movableAgents.end(), i) != movableAgents.end())
                {
                    newVisit++;
                    visited[i][TPGNextIdx] = true;
                    if (TPGNextIdx == this->tpg->getAgent(i)->pathLength - 1)
                    {
                        finishedAgent[i] = true;
                        this->tpg->getAgent(i)->TPGFinishedTime = this->TPGTotalTimeStep;
                    }
                    this->TPGGeneratedPath[i].push_back(TPGNextNode->coord);
                }
                else{
                    this->totalDelay++;
                }
            }
        }
        // 2. Rotations
        newVisit += moveTPGRotation(TPGrotationStopRobots, movableAgents, visited, finishedAgent, NumRobotsCanMoveBasedOnTPG, tempCheckMoveAgents);

        CheckMoveAgents = tempCheckMoveAgents;
        tempCheckMoveAgents.clear();
    }

    if (NumRobotsCanMoveBasedOnTPG == 0)
    {
        // count how many robots are finished
        std::cout << "Deadlock: No robot can move based on TPG: " << std::endl;
        int unfinished = 0;
        for (int i = 0; i < finishedAgent.size(); i++)
        {
            if (!finishedAgent[i])
            {
                unfinished++;
            }
        }
        std::cout << "Unfinished: " << unfinished << std::endl;
        exit(1);
    }

    return;
}

void Sim::SimulateTPGWoDelayTimeStep(std::vector<int> &movableAgents, std::vector<std::vector<bool>> &visited, std::vector<bool> &finishedAgent)
{

    // 1. Check which agent can move based on TPG
    int NumRobotsCanMoveBasedOnTPG = 0;
    std::vector<int> CheckMoveAgents;
    for (int i = 0; i < this->tpg->getNumAgents(); i++)
    {
        CheckMoveAgents.push_back(i);
    }
    std::vector<int> tempCheckMoveAgents;
    std::vector<std::pair<std::pair<int, int>, std::pair<int, int>>> TPGrotationStopRobots;
    int newVisit = 1;
    while (newVisit != 0)
    {
        newVisit = 0;
        TPGrotationStopRobots.clear();
        for (auto i : CheckMoveAgents)
        {
            if (finishedAgent[i])
                continue;
            int TPGNextIdx = std::find(visited[i].begin(), visited[i].end(), false) - visited[i].begin();
            Node *TPGNextNode = this->tpg->getAgent(i)->Type1Next;
            for (int j = 0; j < TPGNextIdx; ++j)
            {
                TPGNextNode = TPGNextNode->Type1Next;
            }
            bool CanVisit = true;

            for (auto edge : TPGNextNode->Type2Prev)
            {

                if (visited[edge->nodeFrom->robotId][edge->nodeFrom->timeStep] == false)
                {
                    CanVisit = false;
                    TPGrotationStopRobots.push_back(std::make_pair(std::make_pair(i, TPGNextIdx), std::make_pair(edge->nodeFrom->robotId, edge->nodeFrom->timeStep)));
                    tempCheckMoveAgents.push_back(i);
                    break;
                }
            }
            if (CanVisit)
            {
                NumRobotsCanMoveBasedOnTPG++;
                newVisit++;
                visited[i][TPGNextIdx] = true;
                if (TPGNextIdx == this->tpg->getAgent(i)->pathLength - 1)
                {
                    finishedAgent[i] = true;
                    this->tpg->getAgent(i)->TPGFinishedTimeNoDelay = this->TPGTotalTimeStepNoDelay;
                }
                this->TPGGeneratedPathNoDelay[i].push_back(TPGNextNode->coord);
            }
        }
        // 2. Rotations
        newVisit += moveTPGWoDelayRotation(TPGrotationStopRobots, movableAgents, visited, finishedAgent, NumRobotsCanMoveBasedOnTPG, tempCheckMoveAgents);

        CheckMoveAgents = tempCheckMoveAgents;
        tempCheckMoveAgents.clear();
    }

    if (NumRobotsCanMoveBasedOnTPG == 0)
    {
        std::cout << "Deadlock: No robot can move based on TPGWoDelay" << std::endl;
        exit(1);
    }

    return;
}

int Sim::moveRotation(std::vector<std::pair<std::pair<int, int>, std::pair<int, int>>> &BTPGrotationStopRobots_, std::vector<int> &movableAgents, std::vector<std::vector<bool>> &visited, std::vector<bool> &finishedAgent, int &NumRobotsCanMoveBasedOnBTPG, std::vector<int> &tempCheckMoveAgents)
{
    int canvisit = 0;
    for (auto pair = BTPGrotationStopRobots_.begin(); pair != BTPGrotationStopRobots_.end(); pair++)
    {
        std::vector<std::pair<int, int>> toVisitBTPGwithGroupsRotation;
        int robotId = pair->first.first;
        int timeStep = pair->first.second;
        int robotId2 = pair->second.first;
        int timeStep2 = pair->second.second;

        toVisitBTPGwithGroupsRotation.push_back(std::make_pair(robotId, timeStep));
        bool find = true;
        bool find2 = false;
        while (find)
        {
            find = false;

            for (auto pair2 = BTPGrotationStopRobots_.begin(); pair2 != BTPGrotationStopRobots_.end(); pair2++)
            {
                // std::cout << "Robot2: " << pair2->first.first << " TimeStep2: " << pair2->first.second << std::endl;
                if (pair2->first.first == robotId2 && pair2->first.second == timeStep2)
                {

                    robotId = pair2->first.first;
                    timeStep = pair2->first.second;
                    robotId2 = pair2->second.first;
                    timeStep2 = pair2->second.second;

                    if (robotId == toVisitBTPGwithGroupsRotation[0].first && timeStep == toVisitBTPGwithGroupsRotation[0].second)
                    {
                        // find = false;

                        find2 = true;
                        break;
                    }

                    if (std::find(toVisitBTPGwithGroupsRotation.begin(), toVisitBTPGwithGroupsRotation.end(), std::make_pair(robotId, timeStep)) != toVisitBTPGwithGroupsRotation.end())
                    {
                        // find = false;
                        // find2 = true;
                        break;
                    }

                    toVisitBTPGwithGroupsRotation.push_back(std::make_pair(robotId, timeStep));
                    find = true;
                    break;
                }
            }
        }

        if (find2)
        {
            bool canVisit = true;
            std::vector<int> CheckBiPair;
            for (auto p : toVisitBTPGwithGroupsRotation)
            {
                // check whether they have other constraints

                int id = p.first;
                int timeStep = p.second;
                Node *nodeBTPG = this->btpg->getAgent(id)->Type1Next;
                for (int j = 0; j < timeStep; j++)
                {
                    nodeBTPG = nodeBTPG->Type1Next;
                }

                for (auto edge : nodeBTPG->Type2Prev)
                {
                    int robotId = edge->nodeFrom->robotId;
                    int time_ = edge->nodeFrom->timeStep;

                    if (std::find(toVisitBTPGwithGroupsRotation.begin(), toVisitBTPGwithGroupsRotation.end(), std::make_pair(robotId, time_)) != toVisitBTPGwithGroupsRotation.end())
                    {
                        continue;
                    }

                    if (this->mode == 1 && edge->isBidirectional)
                    {
                        if (!this->btpg->getBiPair(edge->biPairId)->isVisited)
                        {
                            CheckBiPair.push_back(edge->biPairId);
                            continue;
                        }
                    }

                    if (visited[robotId][time_] == false)
                    {
                        canVisit = false;
                        // std::cout << "Rotation - Robot: "<< id << " " << timeStep << " | " << robotId << " TimeStep: " << time_ << std::endl;
                        break;
                    }
                }
                if (canVisit == false)
                {
                    break;
                }
            }
            if (canVisit)
            {
                NumRobotsCanMoveBasedOnBTPG += toVisitBTPGwithGroupsRotation.size();
                bool canMove = true;
                for (auto p : toVisitBTPGwithGroupsRotation)
                {
                    // check if they can move
                    int id = p.first;
                    if (std::find(movableAgents.begin(), movableAgents.end(), id) == movableAgents.end())
                    {
                        canMove = false;
                    }
                    tempCheckMoveAgents.erase(std::remove(tempCheckMoveAgents.begin(), tempCheckMoveAgents.end(), id), tempCheckMoveAgents.end());
                }
                if (canMove)
                {
                    for (auto p : toVisitBTPGwithGroupsRotation)
                    {
                        // check if they can move
                        canvisit++;
                        int id = p.first;
                        int timeStep = p.second;
                        int nextIdx = std::find(visited[id].begin(), visited[id].end(), false) - visited[id].begin();

                        visited[id][timeStep] = true;
                        if (timeStep == this->btpg->getAgent(id)->pathLength - 1)
                        {
                            finishedAgent[id] = true;
                            this->btpg->getAgent(id)->BTPGFinishedTime = this->BTPGTotalTimeStep;
                        }
                        Node *nodeBTPG = this->btpg->getAgent(id)->Type1Next;
                        for (int j = 0; j < nextIdx; j++)
                        {
                            nodeBTPG = nodeBTPG->Type1Next;
                        }
                        this->BTPGGeneratedPath[id].push_back(nodeBTPG->coord);
                        for (auto biPairId : CheckBiPair)
                        {
                            this->btpg->getBiPair(biPairId)->isVisited = true;
                        }
                    }
                }
            }
            // assert(false);
            break;
        }
    }

    return canvisit;
}

int Sim::moveTPGRotation(std::vector<std::pair<std::pair<int, int>, std::pair<int, int>>> &TPGrotationStopRobots_, std::vector<int> &movableAgents, std::vector<std::vector<bool>> &visited, std::vector<bool> &finishedAgent, int &NumRobotsCanMoveBasedOnTPG, std::vector<int> &tempCheckMoveAgents)
{
    int canvisit = 0;
    for (auto pair = TPGrotationStopRobots_.begin(); pair != TPGrotationStopRobots_.end(); pair++)
    {
        std::vector<std::pair<int, int>> toVisitTPGwithGroupsRotation;
        int robotId = pair->first.first;
        int timeStep = pair->first.second;
        int robotId2 = pair->second.first;
        int timeStep2 = pair->second.second;

        toVisitTPGwithGroupsRotation.push_back(std::make_pair(robotId, timeStep));
        bool find = true;
        bool find2 = false;
        while (find)
        {
            find = false;

            for (auto pair2 = TPGrotationStopRobots_.begin(); pair2 != TPGrotationStopRobots_.end(); pair2++)
            {
                // std::cout << "Robot2: " << pair2->first.first << " TimeStep2: " << pair2->first.second << std::endl;
                if (pair2->first.first == robotId2 && pair2->first.second == timeStep2)
                {

                    robotId = pair2->first.first;
                    timeStep = pair2->first.second;
                    robotId2 = pair2->second.first;
                    timeStep2 = pair2->second.second;

                    if (robotId == toVisitTPGwithGroupsRotation[0].first && timeStep == toVisitTPGwithGroupsRotation[0].second)
                    {
                        // find = false;

                        find2 = true;
                        break;
                    }

                    if (std::find(toVisitTPGwithGroupsRotation.begin(), toVisitTPGwithGroupsRotation.end(), std::make_pair(robotId, timeStep)) != toVisitTPGwithGroupsRotation.end())
                    {
                        // find = false;
                        // find2 = true;
                        break;
                    }

                    toVisitTPGwithGroupsRotation.push_back(std::make_pair(robotId, timeStep));
                    find = true;
                    break;
                }
            }
        }

        if (find2)
        {
            bool canVisit = true;
            for (auto p : toVisitTPGwithGroupsRotation)
            {
                // check whether they have other constraints

                int id = p.first;
                int timeStep = p.second;
                Node *nodeTPG = this->tpg->getAgent(id)->Type1Next;
                for (int j = 0; j < timeStep; j++)
                {
                    nodeTPG = nodeTPG->Type1Next;
                }

                for (auto edge : nodeTPG->Type2Prev)
                {

                    int robotId = edge->nodeFrom->robotId;
                    int time_ = edge->nodeFrom->timeStep;
                    // check this robotId and time_ is in the toVisitTPGwithGroupsRotation
                    if (std::find(toVisitTPGwithGroupsRotation.begin(), toVisitTPGwithGroupsRotation.end(), std::make_pair(robotId, time_)) != toVisitTPGwithGroupsRotation.end())
                    {
                        continue;
                    }

                    if (visited[robotId][time_] == false)
                    {
                        canVisit = false;
                        // std::cout << "Rotation - Robot: "<< id << " " << timeStep << " | " << robotId << " TimeStep: " << time_ << std::endl;
                        break;
                    }
                }
                if (canVisit == false)
                {
                    break;
                }
            }
            if (canVisit)
            {
                NumRobotsCanMoveBasedOnTPG += toVisitTPGwithGroupsRotation.size();
                bool canMove = true;
                for (auto p : toVisitTPGwithGroupsRotation)
                {
                    // check if they can move
                    int id = p.first;
                    if (std::find(movableAgents.begin(), movableAgents.end(), id) == movableAgents.end())
                    {
                        this->totalDelay++;
                        canMove = false;
                    }
                    tempCheckMoveAgents.erase(std::remove(tempCheckMoveAgents.begin(), tempCheckMoveAgents.end(), id), tempCheckMoveAgents.end());
                }
                if (canMove)
                {
                    for (auto p : toVisitTPGwithGroupsRotation)
                    {

                        canvisit++;
                        int id = p.first;
                        int timeStep = p.second;
                        int nextIdx = std::find(visited[id].begin(), visited[id].end(), false) - visited[id].begin();
                        visited[id][timeStep] = true;
                        if (timeStep == this->tpg->getAgent(id)->pathLength - 1)
                        {
                            finishedAgent[id] = true;
                            this->tpg->getAgent(id)->TPGFinishedTime = this->TPGTotalTimeStep;
                        }
                        Node *node = this->tpg->getAgent(id)->Type1Next;
                        for (int j = 0; j < nextIdx; j++)
                        {
                            node = node->Type1Next;
                        }
                        this->TPGGeneratedPath[id].push_back(node->coord);
                    }
                }
            }
            // assert(false);
            break;
        }
    }

    return canvisit;
}

int Sim::moveTPGWoDelayRotation(std::vector<std::pair<std::pair<int, int>, std::pair<int, int>>> &TPGrotationStopRobots_, std::vector<int> &movableAgents, std::vector<std::vector<bool>> &visited, std::vector<bool> &finishedAgent, int &NumRobotsCanMoveBasedOnTPG, std::vector<int> &tempCheckMoveAgents)
{
    int canvisit = 0;
    for (auto pair = TPGrotationStopRobots_.begin(); pair != TPGrotationStopRobots_.end(); pair++)
    {
        std::vector<std::pair<int, int>> toVisitTPGwithGroupsRotation;
        int robotId = pair->first.first;
        int timeStep = pair->first.second;
        int robotId2 = pair->second.first;
        int timeStep2 = pair->second.second;

        toVisitTPGwithGroupsRotation.push_back(std::make_pair(robotId, timeStep));
        bool find = true;
        bool find2 = false;
        while (find)
        {
            find = false;

            for (auto pair2 = TPGrotationStopRobots_.begin(); pair2 != TPGrotationStopRobots_.end(); pair2++)
            {
                // std::cout << "Robot2: " << pair2->first.first << " TimeStep2: " << pair2->first.second << std::endl;
                if (pair2->first.first == robotId2 && pair2->first.second == timeStep2)
                {

                    robotId = pair2->first.first;
                    timeStep = pair2->first.second;
                    robotId2 = pair2->second.first;
                    timeStep2 = pair2->second.second;

                    if (robotId == toVisitTPGwithGroupsRotation[0].first && timeStep == toVisitTPGwithGroupsRotation[0].second)
                    {
                        // find = false;

                        find2 = true;
                        break;
                    }

                    if (std::find(toVisitTPGwithGroupsRotation.begin(), toVisitTPGwithGroupsRotation.end(), std::make_pair(robotId, timeStep)) != toVisitTPGwithGroupsRotation.end())
                    {
                        // find = false;
                        // find2 = true;
                        break;
                    }

                    toVisitTPGwithGroupsRotation.push_back(std::make_pair(robotId, timeStep));
                    find = true;
                    break;
                }
            }
        }

        if (find2)
        {
            bool canVisit = true;
            for (auto p : toVisitTPGwithGroupsRotation)
            {
                // check whether they have other constraints

                int id = p.first;
                int timeStep = p.second;
                Node *nodeTPG = this->tpg->getAgent(id)->Type1Next;
                for (int j = 0; j < timeStep; j++)
                {
                    nodeTPG = nodeTPG->Type1Next;
                }

                for (auto edge : nodeTPG->Type2Prev)
                {
                    int robotId = edge->nodeFrom->robotId;
                    int time_ = edge->nodeFrom->timeStep;

                    if (std::find(toVisitTPGwithGroupsRotation.begin(), toVisitTPGwithGroupsRotation.end(), std::make_pair(robotId, time_)) != toVisitTPGwithGroupsRotation.end())
                    {
                        continue;
                    }

                    if (visited[robotId][time_] == false)
                    {
                        canVisit = false;
                        // std::cout << "Rotation - Robot: "<< id << " " << timeStep << " | " << robotId << " TimeStep: " << time_ << std::endl;
                        break;
                    }
                }
                if (canVisit == false)
                {
                    break;
                }
            }
            if (canVisit)
            {
                NumRobotsCanMoveBasedOnTPG += toVisitTPGwithGroupsRotation.size();
                bool canMove = true;
                if (canMove)
                {
                    for (auto p : toVisitTPGwithGroupsRotation)
                    {
                        canvisit++;
                        int id = p.first;
                        int timeStep = p.second;
                        int nextIdx = std::find(visited[id].begin(), visited[id].end(), false) - visited[id].begin();

                        visited[id][timeStep] = true;
                        if (timeStep == this->tpg->getAgent(id)->pathLength - 1)
                        {
                            finishedAgent[id] = true;
                            this->tpg->getAgent(id)->TPGFinishedTimeNoDelay = this->TPGTotalTimeStepNoDelay;
                        }
                        Node *node = this->tpg->getAgent(id)->Type1Next;
                        for (int j = 0; j < nextIdx; j++)
                        {
                            node = node->Type1Next;
                        }
                        this->TPGGeneratedPathNoDelay[id].push_back(node->coord);
                    }
                }
            }
            // assert(false);
            break;
        }
    }

    return canvisit;
}

void Sim::DecideMovableAgents(std::vector<int> &movableAgents, std::unordered_map<int, int> &robotStopNumbers)
{
    for (int i = 0; i < this->btpg->getNumAgents(); ++i)
    {
        if (std::find(this->DelayedRobots.begin(), this->DelayedRobots.end(), i) != this->DelayedRobots.end())
        {
            double random = (double)rand() / RAND_MAX;

            if (robotStopNumbers[i] != 0)
            {
                robotStopNumbers[i]--;
            }
            else
            {
                if (random > 0.3)
                {
                    movableAgents.push_back(i);
                }
                else
                {
                    robotStopNumbers[i] = 5;
                }
            }
        }
        else
        {
            movableAgents.push_back(i);
        }
    }
}

void Sim::DecideTPGMovableAgents(std::vector<int> &movableAgents, std::unordered_map<int, int> &robotStopNumbers)
{
    for (int i = 0; i < this->tpg->getNumAgents(); ++i)
    {
        if (std::find(this->DelayedRobots.begin(), this->DelayedRobots.end(), i) != this->DelayedRobots.end())
        {
            double random = (double)rand() / RAND_MAX;

            if (robotStopNumbers[i] != 0)
            {
                robotStopNumbers[i]--;
            }
            else
            {
                if (random > 0.3)
                {
                    movableAgents.push_back(i);
                }
                else
                {
                    robotStopNumbers[i] = 5;
                }
            }
        }
        else
        {
            movableAgents.push_back(i);
        }
    }
}

int Sim::GetTPGAverageTime()
{
    return this->TPGaverageTime;
}

int Sim::GetBTPGAverageTime()
{
    return this->BTPGaverageTime;
}

int Sim::GetExpectedDelay()
{
    return this->expectedDelay;
}

int Sim::GetNumBidirectionalEdgesIsUsed()
{
    return this->numBidirectionalEdgesIsUsed;
}