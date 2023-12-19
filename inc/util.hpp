#pragma once
#include <fstream>
#include <iostream>
#include <vector>
#include <cassert>

/**
 * @struct Coord
 * @brief A structure representing a coordinate in 2D space.
 */
struct Coord
{
    int x; ///< The x-coordinate
    int y; ///< The y-coordinate

    /**
     * @brief Default constructor that initializes the coordinates to -1.
     */
    Coord()
    {
        x = -1;
        y = -1;
    };

    /**
     * @brief Constructor that initializes the coordinates to the given values.
     * @param x The x-coordinate.
     * @param y The y-coordinate.
     */
    Coord(int x, int y)
    {
        this->x = x;
        this->y = y;
    };

    /**
     * @brief Checks if this coordinate is equal to another.
     * @param other The other coordinate to compare with.
     * @return True if the coordinates are equal, false otherwise.
     */
    bool operator==(const Coord &other) const
    {
        return x == other.x && y == other.y;
    }

    /**
     * @brief Checks if this coordinate is not equal to another.
     * @param other The other coordinate to compare with.
     * @return True if the coordinates are not equal, false otherwise.
     */
    bool operator!=(const Coord &other) const
    {
        return !(*this == other);
    }
    // hash function for unordered_map
    struct Hash
    {
        std::size_t operator()(const Coord &v) const
        {
            return std::hash<int>()(v.x) ^ std::hash<int>()(v.y) << 1;
        }
    };
};

struct type2Edge;
/**
 * @struct Node
 * @brief A structure representing a node in the TPG.
 */
struct Node
{
    Coord coord;                        ///< The coordinates of the Node in the map
    Node *Type1Next;                    ///< Pointer to the next Node of type 1
    std::vector<type2Edge *> Type2Next; ///< Vector of pointers to the next Nodes of type 2
    Node *Type1Prev;                    ///< Pointer to the previous Node of type 1
    std::vector<type2Edge *> Type2Prev; ///< Vector of pointers to the previous Nodes of type 2
    int timeStep;                       ///< The time step at which this Node exists
    int robotId;                        ///< The ID of the robot at this Node
    /**
     * @brief Default constructor for Node.
     */
    Node()
    {
        coord = Coord();
        Type1Next = NULL;
        Type1Prev = NULL;
        Type2Next = std::vector<type2Edge *>();
        Type2Prev = std::vector<type2Edge *>();
        timeStep = -1;
        robotId = -1;
    };
    Node(int x, int y)
    {
        coord = Coord(x, y);
        Type1Next = NULL;
        Type1Prev = NULL;
        Type2Next = std::vector<type2Edge *>();
        Type2Prev = std::vector<type2Edge *>();
        this->timeStep = -1;
        this->robotId = -1;
    };

    // overwrite << operator
    friend std::ostream &operator<<(std::ostream &os, const Node &obj)
    {
        os << "(" << obj.coord.x << "," << obj.coord.y << ")";
        return os;
    }
};

/**
 * @struct Agent
 * @brief A structure representing an agent in the TPG.
 */
struct Agent
{
    Node *Type1Next;  ///< Pointer to the first Node of type 1
    int pathLength;   ///< The length of the path of the agent
    int robotId;      ///< The ID of the robot
    int finishedTime; ///< The time step at which the agent finished
    // Simulation
    int BTPGFinishedTime = -1; ///< The time step at which the agent finished in the BTPG
    int TPGFinishedTime = -1;  ///< The length of the path of the agent in the BTPG
    int TPGFinishedTimeNoDelay = -1;
    Agent()
    {
        Type1Next = NULL;
        pathLength = 0;
        robotId = -1;
        finishedTime = -1;
    };
    Agent(int robotId, Node *Type1Next)
    {
        Type1Next = Type1Next;
        pathLength = 0;
        this->robotId = robotId;
        finishedTime = -1;
    };
};

/**
 * @struct type2Edge
 * @brief A structure representing a type 2 edge in the TPG.
 */
struct type2Edge
{
    int edgeId; ///< The ID of the edge
    int biPairId;
    int groupId; ///< The ID of the group that this edge belongs to
    int biGroupId;

    Node *nodeFrom; ///< Pointer to the Node from which this edge originates
    Node *nodeTo;   ///< Pointer to the Node to which this edge leads

    bool isBidirectional; ///< True if this edge is bidirectional, false otherwise
    bool isGroupedBidirectional;
    bool isGrouped; ///< True if this edge is grouped, false otherwise
    /**
     * @brief Default constructor for type2Edge.
     */
    type2Edge()
    {
        nodeFrom = NULL;
        nodeTo = NULL;
        edgeId = -1;
        groupId = -1;
        biPairId = -1;
        biGroupId = -1;
        isBidirectional = false;
        isGroupedBidirectional = false;
        isGrouped = false;
    };
};

/**
 * @struct Type2EdgeGroup
 * @brief A structure representing a group of type 2 edges in the TPG.
 */
struct Type2EdgeGroup
{
    std::vector<type2Edge *> type2Edges;
    int groupId;
    bool canBeReversed = false;
    int fromId;
    int toId;
    type2Edge *earliestOutEdge = nullptr;
    type2Edge *earliestInEdge = nullptr;
    bool isBidirectional = false;
    int biGroupId = -1;
    Type2EdgeGroup()
    {
        groupId = -1;
        fromId = -1;
        toId = -1;
    };
};

/**
 * @struct BiPair
 * @brief A structure representing Bidirectional Pairs in the BTPG.
 */
struct BiPair
{
    int id;         ///< The ID of the BiPair
    int originalId; ///< The ID of original edge in the pair
    int flippedId;  ///< The ID of flipped edge in the pair

    // Simulation
    bool isVisited = false; ///< True if this BiPair has been visited, false otherwise

    /**
     * @brief Constructor that initializes the BiPair with the given original and flipped IDs.
     * @param first The original ID.
     * @param second The flipped ID.
     */
    BiPair(int first, int second)
    {
        this->originalId = first;
        this->flippedId = second;
    }

    /**
     * @brief Default constructor that initializes the original and flipped IDs to -1.
     */
    BiPair()
    {
        this->originalId = -1;
        this->flippedId = -1;
    }
};

/**
 * @struct BiGroupPair
 * @brief A structure representing Bidirectional Group Pairs in the BTPG.
 */
struct BiGroupPair
{
    int id;         ///< The ID of the BiPair
    int originalId; ///< The ID of original group in the pair
    int flippedId;  ///< The ID of flipped group in the pair

    // Simulation
    bool isVisited = false; ///< True if this BiPair has been visited, false otherwise

    /**
     * @brief Constructor that initializes the BiPair with the given original and flipped IDs.
     * @param first The original ID.
     * @param second The flipped ID.
     */
    BiGroupPair(int first, int second)
    {
        this->originalId = first;
        this->flippedId = second;
    }

    /**
     * @brief Default constructor that initializes the original and flipped IDs to -1.
     */
    BiGroupPair()
    {
        this->originalId = -1;
        this->flippedId = -1;
    }
};