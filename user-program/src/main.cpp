#include "cmdline.h"
#include "bandit/init_util.hpp"

using namespace std;
using namespace bandit;

int main(int argc, char* argv[])
{
    cmdline::parser cmd;
    // add specified type of variable.
    // 1st argument is long name
    // 2nd argument is short name (no short name if '\0' specified)
    // 3rd argument is description
    // 4th argument is mandatory (optional. default is false)
    // 5th argument is default value  (optional. it used when mandatory is false)
    cmd.add<uint>("times", 'n', "times of simulations", false, 1);
    cmd.add<uint>("M", 'M', "M paths", false, 2);
    cmd.add<uint>("rounds", 'T', "number of rounds in a simulation", false, 10000);
    cmd.add<string>("file", 'f', "filename for parameters", false, "./pathdata/paraFile.txt");
    cmd.add<string>("output", 'o', "output filename", false, "pathLog.txt");
    cmd.add<double>("threshold", 'h', "threshold for ConMPTSLatency", false, 0.4);
    // Use the damping factor with care.
    // The larger the factor the more forgetful the algorithm,
    // but the more unstable estimation of the algorithm.
    cmd.add<uint>("Delta", 'D', "Delta_t: interval for the simulator (us)", false, 100);
    // this is to test the flow completion time, if forever, then no logging
    // the program terminates if there is no measurement.
    cmd.add<bool>("Forever", 'F', "Forever running until the end", false, false);
    cmd.add<double>("damping", 'd', "damping factor for ConMPTSLatency", false, 0.01);
    cmd.add<int>("seed", 's', "random number seed", false, -1);
#ifdef OLMS_KERNEL
    cmd.add<uint>("P", 'P', "Total P paths", true, 2);
    cmd.add<string>("pathtype", 'p', "Path type: < bernoulli | kernel >", true, "bernoulli");
    cmd.add<uint>("maxrtt", 'r', "The upper bound of RTprop (ms)", false, 100);
    cmd.add<uint>("maxbtlbw", 'b', "The uppper bound of BtlBw (Mbit per second)", false, 100);
#endif
    cmd.parse_check(argc, argv);
    const uint n = cmd.get<uint>("times");
    const uint M = cmd.get<uint>("M");
    const uint T = cmd.get<uint>("rounds");
    const string parasFile = cmd.get<string>("file");
    const string outputFile = cmd.get<string>("output");
    const double threshold = cmd.get<double>("threshold");
    const uint Delta_t = cmd.get<uint>("Delta");
    const bool isForever = cmd.get<bool>("Forever");
    const double damping_factor = cmd.get<double>("damping");
    int rngSeed = cmd.get<int>("seed");
    if (rngSeed!=-1) {
        cout << "rngSeed=" << rngSeed << endl;
        randomEngine = std::mt19937(rngSeed);
    }
#ifdef OLMS_KERNEL
    const uint num_paths = cmd.get<uint>("P");
    const string pathType = cmd.get<string>("pathtype");
    // In the kernel: srtt is scaled up to 8 times of the true value i us
    const uint max_rtt = (cmd.get<uint>("maxrtt")*1000);     // change to scale the srtt
    // In the kernel: bw is scaled down to Bytes per second
    const uint max_btlbw = (cmd.get<uint>("maxbtlbw")*1000000) >> 3; // change to bytes
    kcmpts.set_num_paths(num_paths);
    kcmpts.set_max_rtt(max_rtt);
    kcmpts.set_max_btlbw(max_btlbw);
#endif
    vector<PathPtr> paths;
    vector<PolicyPtr> policies;

#ifdef OLMS_KERNEL
    if (pathType.compare("bernoulli")==0) {
        initPaths(paths, parasFile);
    }
    else if (pathType.compare("kernel")==0) {
        initKernelPaths(paths, M, num_paths);
    }
#else
    initPaths(paths, parasFile);
#endif
    cout << "Initpath finished..." << endl;
    initPolicies(policies, paths.size(), threshold, damping_factor);
#if DEBUG_mode
    for (auto p : policies) {
        std::cout << "main.cpp: " << p->info() << "" << std::endl;
    }
#endif
    cout << "Initpolicies finished..." << endl;
//    bool recommendSinglePath = false;
    startSimulation(n, T, M, threshold, paths, policies, outputFile, Delta_t, isForever);

    return 0;
}


