
// #include "TPG.hpp"
// #include "BTPG.hpp"
// #include "BTPGWithGroup.hpp"
#include "Sim.hpp"

int main(int argc, char *argv[])
{
    // read filename from input arg
    std::string filename;
    int seed;
    int algorithmIdx;
    int timeInterval;

    for (int i = 1; i < argc; ++i)
    {
        std::string arg = argv[i];

        // Check for different options
        if (arg == "-h" || arg == "--help")
        {
            std::cout << "Usage: ./CompareTPGandBTPG -f <filename> -s <seed> -a <algorithmIdx>" << std::endl;
        }
        else if (arg == "-v" || arg == "--version")
        {
            std::cout << "Version 1.0" << std::endl;
        }
        else if (arg == "-f" || arg == "--file")
        {
            // Access the value of the option
            if (i + 1 < argc)
            {
                filename = argv[i + 1];
                std::cout << "File: " << filename << std::endl;
                ++i; // Skip the next argument as it's the value
            }
            else
            {
                std::cerr << "No file name provided!" << std::endl;
                return 1;
            }
        }
        else if (arg == "-s" || arg == "--seed")
        {
            if (i + 1 < argc)
            {
                seed = std::stoi(argv[i + 1]);
                ++i;
            }
            else
            {
                std::cerr << "No seed provided!" << std::endl;
                return 1;
            }
        }
        else if (arg == "-a" || arg == "--algorithm")
        {
            if (i + 1 < argc)
            {
                algorithmIdx = std::stoi(argv[i + 1]);
                ++i;
            }
            else
            {
                std::cerr << "No seed provided!" << std::endl;
                return 1;
            }
        }
        else if (arg == "-t" || arg == "--time")
        {
            if (i + 1 < argc)
            {
                timeInterval = std::stoi(argv[i + 1]);
                ++i;
            }
            else
            {
                std::cerr << "No timeInterval provided!" << std::endl;
                return 1;
            }
        }
        else
        {
            std::cerr << "Unknown option: " << arg << std::endl;
            return 1;
        }
    }
    int singleTimeInterval = timeInterval;
    bool BTPGFinished = false;
    while (!BTPGFinished)
    {
        TPG *tpg = new TPG(filename);
        BTPG *btpg = new BTPG(filename, algorithmIdx, timeInterval);
        // TPG *tpg = new TPG("./test/100.txt");
        // BTPG *btpg = new BTPG("./test/100.txt", 0);

        Sim *sim = new Sim(seed, btpg->getNumAgents());

        int result = sim->Simulate(tpg);
        result = sim->Simulate(btpg);
        sim->SimulateNoDelay(tpg);
        std::cout << "BTPG Bi-Type2 used:" << sim->GetNumBidirectionalEdgesIsUsed() << std::endl;

        // // calculate the statistics
        double improvement = (double)(sim->GetTPGAverageTime() - sim->GetBTPGAverageTime()) / (sim->GetTPGAverageTime() - sim->GetExpectedDelay());
        std::cout << "TPG vs BTPG-o improvement: " << improvement << std::endl;
        if (btpg->finish)
        {
            BTPGFinished = true;
        }
        else
        {
            timeInterval += timeInterval;
        }
    }

    return 0;
}