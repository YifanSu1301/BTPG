#include "BTPG.hpp"
#include <boost/property_tree/ptree.hpp>
#include "Sim.hpp"

const int BTPG_n = 0;
const int BTPG_o = 1;

BTPG::BTPG(std::string fileName, int mode, int timeInterval)
    : TPG(fileName)
{
    this->mode = mode;
    this->numBiPairs = 0;
    this->naiveNegativeCase = 0;
    // Grouping
#ifdef DEBUG
    std::cout << "| BTPG constructor |" << std::endl;
    std::cout << "Start Grouping ..." << std::endl;
#endif
    for (int i = 0; i < getNumAgents(); i++)
    {
        Agent *agent = getAgent(i);
        Node *node = agent->Type1Next;

        while (node != NULL)
        {
            if (node->Type2Next.size() > 0)
            {
                for (auto &edge : node->Type2Next)
                {
                    if (edge->isGrouped)
                        continue;
                    int toId = edge->nodeTo->robotId;
                    int timeStep = edge->nodeTo->timeStep;
                    Node *fromNode = node;
                    Type2EdgeGroup *group = new Type2EdgeGroup();
                    int groupId = getNumType2EdgeGroups();
                    group->fromId = fromNode->robotId;
                    fromNode = fromNode->Type1Next;
                    group->type2Edges.push_back(edge);
                    group->toId = toId;
                    edge->isGrouped = true;
                    edge->groupId = groupId;

                    while (fromNode != NULL)
                    {
                        bool found = false;
                        for (auto &edge2 : fromNode->Type2Next)
                        {
                            if (edge2->isGrouped)
                                continue;
                            if (edge2->nodeTo->robotId == toId && (edge2->nodeTo->timeStep == timeStep + 1 || edge2->nodeTo->timeStep == timeStep - 1))
                            {
                                group->type2Edges.push_back(edge2);
                                edge2->isGrouped = true;
                                edge2->groupId = groupId;
                                timeStep = edge2->nodeTo->timeStep;
                                found = true;
                                break;
                            }
                        }
                        if (!found)
                            break;
                        fromNode = fromNode->Type1Next;
                    }

                    addType2EdgeGroup(group);
                }
            }
            node = node->Type1Next;
        }
    }

#ifdef DEBUG
    std::cout << "End Grouping." << std::endl;
    std::cout << "******** Grouping Info ********" << std::endl;
    std::cout << "Number of groups: " << getNumType2EdgeGroups() << std::endl;
    std::cout << "Start BTPG ..." << std::endl;
#endif
    // count time
    auto start = std::chrono::high_resolution_clock::now();
    int type2EdgeSigleton = 0;
    int addMorePairs = 1;
    while (addMorePairs != 0)
    {
        addMorePairs = 0;
        type2EdgeSigleton = 0;
        for (int i = 0; i < getNumType2EdgeGroups(); i++)
        {
            Type2EdgeGroup *group = getType2EdgeGroup(i);
            if (group->type2Edges.size() > 1)
            {
                // CheckGroup(group) //TODO: Next Step
                continue;
            }
            else
            {
                auto endAnytime = std::chrono::high_resolution_clock::now();
                auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(endAnytime - start).count();
                if (timeInterval != 0 && duration > timeInterval)
                {

#ifdef DEBUG
                    std::cout << "End BTPG." << std::endl;
                    std::cout << "******** BTPG Info ********" << std::endl;
                    std::cout << "Number of type-2 edge sigleton: " << type2EdgeSigleton << std::endl;
                    std::cout << "Number of BiPairs: " << getNumBiPairs() << std::endl;
                    std::cout << "Number of naive negative cases: " << this->naiveNegativeCase << std::endl;
                    std::cout << "******** ***** ********" << std::endl;
#endif
                    return;
                }
                type2EdgeSigleton++;
                int biPairNum = getNumBiPairs();
                if (group->type2Edges[0]->isBidirectional)
                    continue;
                CheckSingleton(group->type2Edges[0]);
                if (biPairNum != getNumBiPairs())
                {
                    addMorePairs++;
                }
            }
        }
#ifdef DEBUG
        std::cout << "current loop:" << getNumBiPairs() << std::endl;
#endif
        if (this->mode == 0)
        {
            addMorePairs = 0;
        }
    }

    auto end = std::chrono::high_resolution_clock::now();
    this->finish = true;
#ifdef DEBUG
    std::cout << "End BTPG." << std::endl;
    std::cout << "******** BTPG Info ********" << std::endl;
    std::cout << "Number of type-2 edge sigleton: " << type2EdgeSigleton << std::endl;
    std::cout << "Number of BiPairs: " << getNumBiPairs() << std::endl;
    std::cout << "Number of naive negative cases: " << this->naiveNegativeCase << std::endl;
    std::cout << "Time taken: " << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() << "ms" << std::endl;
    std::cout << "******** ***** ********" << std::endl;
#endif
// output the BTPG into a json file using boost library
#ifdef DEBUG
    std::cout << "Start output BTPG ..." << std::endl;
#endif
}

int BTPG::getNumType2EdgeGroups()
{
    return this->Type2EdgeGroups.size();
}

void BTPG::addType2EdgeGroup(Type2EdgeGroup *group)
{
    this->Type2EdgeGroups.push_back(group);
}

Type2EdgeGroup *BTPG::getType2EdgeGroup(int groupId)
{
    return this->Type2EdgeGroups[groupId];
}

void BTPG::addBiPair(BiPair *pair)
{
    this->BiPairs.push_back(pair);
}

int BTPG::getNumBiPairs()
{
    return this->BiPairs.size();
}

BiPair *BTPG::getBiPair(int biPairId)
{
    return this->BiPairs[biPairId];
}

// Helper functions
void BTPG::CheckSingleton(type2Edge *candidateEdge)
{
    if (candidateEdge->isBidirectional)
        return;
    if (CheckSingletonValidity(candidateEdge))
    {
        // set candidateEdge to be bidirectional
        candidateEdge->isBidirectional = true;

        // Add another type-2 edge
        type2Edge *newType2Edge = new type2Edge();
        newType2Edge->nodeFrom = candidateEdge->nodeTo;
        newType2Edge->nodeTo = candidateEdge->nodeFrom;
        newType2Edge->edgeId = getNumTypeTwoEdges();
        newType2Edge->isBidirectional = true;
        addTypeTwoEdge(newType2Edge);

        // Add new edge to the node
        candidateEdge->nodeTo->Type1Next->Type2Next.push_back(newType2Edge);
        candidateEdge->nodeFrom->Type1Prev->Type2Prev.push_back(newType2Edge);

        // Add BiPair
        BiPair *newBiPair = new BiPair(candidateEdge->edgeId, newType2Edge->edgeId);
        newBiPair->id = getNumBiPairs();
        addBiPair(newBiPair);

        // Update two edges
        candidateEdge->biPairId = newBiPair->id;
        newType2Edge->biPairId = newBiPair->id;
    }
    return;
}

bool BTPG::CheckSingletonValidity(type2Edge *candidateEdge)
{
    // std::cout << candidateEdge->nodeFrom->robotId << " " << candidateEdge->nodeFrom->timeStep << " -> " << candidateEdge->nodeTo->robotId << " " << candidateEdge->nodeTo->timeStep << std::endl;
    // 1. Initialization
    // 1a. Initialize the start and end nodes
    Node *startNode = candidateEdge->nodeFrom->Type1Prev;
    Node *endNode = candidateEdge->nodeTo->Type1Next;
    // edge cases
    if (endNode == NULL || startNode->timeStep == 0)
    {
        this->naiveNegativeCase++;
        return false;
    }

    // Temporarily set the edge to be bidirectional
    candidateEdge->isBidirectional = true;
    // Temporarily add a new edge to type-2 edge
    type2Edge *newType2Edge = new type2Edge();
    newType2Edge->nodeFrom = endNode;
    newType2Edge->nodeTo = startNode;
    newType2Edge->edgeId = getNumTypeTwoEdges();
    newType2Edge->isBidirectional = true;
    addTypeTwoEdge(newType2Edge);

    // 1b. Initialize the visit stacks for nodes
    std::set<Node *> visitedNodes;

    // 1b'. Initialize the visit stacks for current node
    Node *setVisitedNode = endNode->Type1Prev;
    while (setVisitedNode != NULL)
    {
        visitedNodes.insert(setVisitedNode);
        setVisitedNode = setVisitedNode->Type1Prev;
    }

    // 1c. Initialize the stacks for revisit nodes
    std::vector<std::set<Node *>> revisitNodes;
    for (int i = 0; i < getNumTypeTwoEdges(); i++)
    {
        std::set<Node *> temp;
        revisitNodes.push_back(temp);
    }

    // 1d. stack for current recursion path
    std::vector<Node *> recursionPath;
    std::set<Node *> recursionPathSet;

    // 1e. stack for edges in the current recursion path
    std::set<int> recursionEdgePath;

    // 1f. dictionary for agents that how to get to the end node
    std::unordered_map<int, int> agentEdgeMap;
    for (int i = 0; i < getNumAgents(); i++)
    {
        agentEdgeMap[i] = -1;
    }

    // 1g. dictionary for agents that how to leave to the end node
    std::unordered_map<int, int> agentEdgeMapLeave;
    for (int i = 0; i < getNumAgents(); i++)
    {
        agentEdgeMapLeave[i] = -1;
    }

    agentEdgeMap[startNode->robotId] = candidateEdge->edgeId;
    // std::cout << "StartNode: " << startNode->robotId << " " << startNode->timeStep << std::endl;
    // check if the startnode has a type-2 neighbor is 12 99
    // for (auto &edge : startNode->Type2Next)
    // {
    //     std::cout << "edge: " << edge->nodeTo->robotId << " " << edge->nodeTo->timeStep << std::endl;
    // }

    agentEdgeMap[startNode->robotId] = newType2Edge->edgeId;
    // 2. Start the search
    if (BidirectionalDFS(startNode, endNode, visitedNodes, revisitNodes, recursionPath, recursionPathSet, agentEdgeMap, agentEdgeMapLeave, false))
    {
        candidateEdge->isBidirectional = false;
        // delete the new edge
        removeTypeTwoEdge(newType2Edge);
        delete newType2Edge;
        return false;
    }
    else
    {
        candidateEdge->isBidirectional = false;
        // delete the new edge
        removeTypeTwoEdge(newType2Edge);
        delete newType2Edge;
        return true;
    }
}

bool BTPG::BidirectionalDFS(Node *currNode_, Node *endNode_, std::set<Node *> &VisitedStack_, std::vector<std::set<Node *>> &RevisitedNodes_, std::vector<Node *> &RecursionPath_, std::set<Node *> &RecursionPathSet_,
                            std::unordered_map<int, int> &AgentEdgeMap_, std::unordered_map<int, int> &AgentEdgeMapLeave_, bool hasTYpe1Edge_)
{
    // std::cout << "currNode: " << currNode_->robotId << " " << currNode_->timeStep << std::endl;
    // !: Base Case
    // 1. check if reach the end node
    if (currNode_ == endNode_)
    {
        if (hasTYpe1Edge_ || RecursionPath_.size() > 2)
        {
            // print out the recursion path
            // std::cout << "Recursion Path: ";
            // for (auto &node : RecursionPath_)
            // {
            //     std::cout << node->robotId << " " << node->timeStep << " -> ";
            // }
            // std::cout << std::endl;
            // std::cout << "endNode: " << endNode_->robotId << " " << endNode_->timeStep << std::endl;
            return true;
        }
        else
        {
            // revisit all the nodes in the recursion path
            for (auto node = RecursionPath_.rbegin(); node != RecursionPath_.rend() - 1; node++)
            {
                // ??: should set unvisit directly
                VisitedStack_.erase(*node);
            }
            return false;
        }
    }

    // Update
    VisitedStack_.insert(currNode_);
    RecursionPath_.push_back(currNode_);
    RecursionPathSet_.insert(currNode_);

    // !: Traverse Type-2 Edge
    for (auto &edge : currNode_->Type2Next)
    {
        // 2. Checking if need to keep going on
        if (VisitedStack_.find(edge->nodeTo) == VisitedStack_.end() && RecursionPathSet_.find(edge->nodeTo) == RecursionPathSet_.end())
        {
            // Need keep visiting next node
            // 2a. if so, checking if the edge is bidirectional
            if (edge->isBidirectional)
            {
                if (this->mode == 0 && AgentEdgeMap_[currNode_->robotId] != -1)
                {
                    // Only check if the previous edge is the same bidirectional edge
                    Node *prevNode = getTypeTwoEdge(AgentEdgeMap_[currNode_->robotId])->nodeTo;
                    if (edge->biPairId == getTypeTwoEdge(AgentEdgeMap_[currNode_->robotId])->biPairId)
                    {
                        // !: if reach current Node by a type-1 edge, then we should only revisit current node
                        if (RecursionPath_.back()->robotId == currNode_->robotId)
                        {
                            RevisitedNodes_[AgentEdgeMap_[currNode_->robotId]].insert(currNode_);
                        }
                        else
                        {
                            // !: if reach current Node by a type-2 edge,
                            // !then we should revisit all the nodes in the recursion path back to the prevNode
                            for (auto renode = RecursionPath_.rbegin(); renode != RecursionPath_.rend(); renode++)
                            {
                                RevisitedNodes_[AgentEdgeMap_[currNode_->robotId]].insert(*renode);
                                if (*renode == prevNode)
                                {
                                    break;
                                }
                            }
                        }
                        continue;
                    }
                }
                // 2a.1: Check if we should visit the edge (BTPG-o)
                if (this->mode == 1 && AgentEdgeMap_[currNode_->robotId] != -1)
                {
                    // Have visited the same agent
                    // std::cout << "current number of type-2:" << getNumTypeTwoEdges() << std::endl;
                    // std::cout << "current number of type-2:" << AgentEdgeMap_[currNode_->robotId] << std::endl;
                    Node *prevNode = getTypeTwoEdge(AgentEdgeMap_[currNode_->robotId])->nodeTo;
                    // 2a.1.1: Check if prevNode is in the same robot's path and has smaller time step
                    if (prevNode->timeStep < currNode_->timeStep)
                    {
                        // !: if reach current Node by a type-1 edge, then we should only revisit current node
                        if (RecursionPath_.back()->robotId == currNode_->robotId)
                        {
                            RevisitedNodes_[AgentEdgeMap_[currNode_->robotId]].insert(currNode_);
                        }
                        else
                        {
                            // !: if reach current Node by a type-2 edge,
                            // !then we should revisit all the nodes in the recursion path back to the prevNode
                            for (auto renode = RecursionPath_.rbegin(); renode != RecursionPath_.rend(); renode++)
                            {
                                RevisitedNodes_[AgentEdgeMap_[currNode_->robotId]].insert(*renode);
                                if (*renode == prevNode)
                                {
                                    break;
                                }
                            }
                        }

                        continue;
                    }
                }
            }

            // 2a.2: Check if we should visit the edge (BTPG-o)
            if (this->mode == 1 && AgentEdgeMapLeave_[edge->nodeTo->robotId] != -1)
            {
                type2Edge *prevEdge = getTypeTwoEdge(AgentEdgeMapLeave_[edge->nodeTo->robotId]);
                Node *prevFromNode = prevEdge->nodeFrom;
                if (prevFromNode->timeStep > edge->nodeTo->timeStep && prevEdge->isBidirectional)
                {
                    for (auto renode = RecursionPath_.rbegin(); renode != RecursionPath_.rend(); renode++)
                    {
                        RevisitedNodes_[AgentEdgeMapLeave_[edge->nodeTo->robotId]].insert(*renode);
                        if (*renode == prevEdge->nodeTo)
                        {
                            break;
                        }
                    }
                    continue;
                }
            }

            // 2b. keep visiting next node

            if (edge->nodeTo->robotId != -1)
            {
                // 2b.1: update agentEdgeMap (enter)
                AgentEdgeMap_[edge->nodeTo->robotId] = edge->edgeId;
            }
            // 2b.2: update agentEdgeMap (leave)
            AgentEdgeMapLeave_[currNode_->robotId] = edge->edgeId;

            // !: c.Recursion
            if (BidirectionalDFS(edge->nodeTo, endNode_, VisitedStack_, RevisitedNodes_, RecursionPath_, RecursionPathSet_, AgentEdgeMap_, AgentEdgeMapLeave_, hasTYpe1Edge_))
            {
                return true;
            }

            if (edge->nodeTo->robotId != -1)
            {
                // 2b.3: update agentEdgeMap (leave), without the edge in the recursion path
                AgentEdgeMap_[edge->nodeTo->robotId] = -1;
            }
            // 2b.4: update agentEdgeMap (leave), without the edge in the recursion path
            AgentEdgeMapLeave_[currNode_->robotId] = -1;

            // revisit relevant nodes
            for (auto &node : RevisitedNodes_[edge->edgeId])
            {
                VisitedStack_.erase(node);
            }
            RevisitedNodes_[edge->edgeId].clear();
        }
        else
        {
            continue;
        }
    }

    // !: Traverse Type-1 Edge
    if (currNode_->Type1Next != NULL)
    {
        // 1. check if need to keep going on
        if (VisitedStack_.find(currNode_->Type1Next) == VisitedStack_.end() && RecursionPathSet_.find(currNode_->Type1Next) == RecursionPathSet_.end())
        {
            // !: c.Recursion
            if (BidirectionalDFS(currNode_->Type1Next, endNode_, VisitedStack_, RevisitedNodes_, RecursionPath_, RecursionPathSet_, AgentEdgeMap_, AgentEdgeMapLeave_, true))
            {
                return true;
            }
        }
    }

    // Update
    RecursionPath_.pop_back();
    RecursionPathSet_.erase(currNode_);

    return false;
}