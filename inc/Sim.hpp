#pragma once
#include "util.hpp"
#include "BTPG.hpp"
// #include "BTPGWithGroup.hpp"

class Sim
{
private:
    BTPG *btpg = nullptr;
    TPG *tpg = nullptr;
    int seed;
    bool isBTPG = false;
    int mode = 0;
    int expectedDelay = 0;
    int BTPGaverageTime = 0;
    int TPGaverageTime = 0;

    int numBidirectionalEdgesIsUsed = 0;

    int totalDelay = 0;

    std::vector<int> DelayedRobots;
    std::vector<std::vector<Coord>> BTPGGeneratedPath;
    std::vector<std::vector<Coord>> TPGGeneratedPath;
    std::vector<std::vector<Coord>> TPGGeneratedPathNoDelay;
    int BTPGTotalTimeStep;
    int TPGTotalTimeStep;
    int TPGTotalTimeStepNoDelay;

    void DecideMovableAgents(std::vector<int> &movableAgents, std::unordered_map<int, int> &robotStopNumbers);
    void DecideTPGMovableAgents(std::vector<int> &movableAgents, std::unordered_map<int, int> &robotStopNumbers);

    void SimulateTimeStep(std::vector<int> &movableAgents, std::vector<std::vector<bool>> &visited, std::vector<bool> &finishedAgent);
    void SimulateTPGTimeStep(std::vector<int> &movableAgents, std::vector<std::vector<bool>> &visited, std::vector<bool> &finishedAgent);
    void SimulateTPGWoDelayTimeStep(std::vector<int> &movableAgents, std::vector<std::vector<bool>> &visited, std::vector<bool> &finishedAgent);

    int moveRotation(std::vector<std::pair<std::pair<int, int>, std::pair<int, int>>> &BTPGrotationStopRobots_, std::vector<int> &movableAgents, std::vector<std::vector<bool>> &visited, std::vector<bool> &finishedAgent, int &NumRobotsCanMoveBasedOnBTPG, std::vector<int> &tempCheckMoveAgents);
    int moveTPGRotation(std::vector<std::pair<std::pair<int, int>, std::pair<int, int>>> &BTPGrotationStopRobots_, std::vector<int> &movableAgents, std::vector<std::vector<bool>> &visited, std::vector<bool> &finishedAgent, int &NumRobotsCanMoveBasedOnBTPG, std::vector<int> &tempCheckMoveAgents);
    int moveTPGWoDelayRotation(std::vector<std::pair<std::pair<int, int>, std::pair<int, int>>> &TPGrotationStopRobots_, std::vector<int> &movableAgents, std::vector<std::vector<bool>> &visited, std::vector<bool> &finishedAgent, int &NumRobotsCanMoveBasedOnTPG, std::vector<int> &tempCheckMoveAgents);

    void CheckBTPGPathValidity();
    void CheckTPGPathValidity();
    void CheckTPGWoDelayPathValidity();

    void GetStatistics();
    void GetTPGStatistics();
    void GetTPGWoDelayStatistics();

public:
    Sim(int seed, int numRobots);

    int Simulate(BTPG *btpg_);
    int Simulate(TPG *tpg);
    void SimulateNoDelay(TPG *tpg_);
    int GetTPGAverageTime();
    int GetBTPGAverageTime();
    int GetExpectedDelay();
    int GetNumBidirectionalEdgesIsUsed();
};