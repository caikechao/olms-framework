#pragma once

#include "macro_util.h"
#include "../path/path.hpp"
#include "../path/path_bernoulli.hpp"
#include "../path/path_fixvalue.hpp"
#include "../path/path_kernel.hpp"
#include "../policy/policy.hpp"
#include "../policy/policy_conmpts_latency.hpp"
#include "../policy/policy_mpts.hpp"
#include "../policy/policy_klucb.hpp"
#include "../policy/policy_exp3m.hpp"
#include "../policy/policy_random.hpp"
#include "bandit_util.hpp"
#ifdef OLMS_KERNEL
#include "kernel_util.hpp"
#endif
#include "simulator.hpp"

#include <iostream>
#include <thread>
#include <chrono>

namespace bandit {

std::vector<Metric> initPathParameters(std::string filename)
{
    std::cout << "# Initializing path parameters from " << filename << std::endl;
    std::vector<Metric> pathPara;
    std::ifstream dataFile(filename);
    std::string line;
    double r, b, l;
    while (std::getline(dataFile, line)) {
        if (line.at(0)=='#') // do not read the comments in the file
            continue;
        std::istringstream(line, std::ios_base::in) >> r >> b >> l;
        pathPara.emplace_back(Metric(r, b, l));
    }
    dataFile.close();
    std::cout << "# Finish initialization of parameters " << pathPara.size() << " paths." << std::endl;
    return pathPara;
}

// initialize the paths
void initPaths(std::vector<PathPtr>& paths, const std::string& paraFile)
{
    paths.clear();
    std::vector<Metric> pathParas = initPathParameters(paraFile);
    for (const auto& pathPara : pathParas) {
        paths.push_back(PathPtr(new FixValuePath(pathPara))); // change the path type
    }
}

#ifdef OLMS_KERNEL
void initKernelPaths(std::vector<PathPtr>& paths, uint M, uint num_paths)
{
    int ret;

    // wait for enough number of paths
    std::cout << "# Waiting for target MPTCP flow " << kcmpts.getNumPaths()
              << std::endl;
    while ((ret = kcmpts.getNumPaths())<num_paths) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    std::cout << "# Num paths: " << kcmpts.getNumPaths() << std::endl;

    for (int i = 0; i<num_paths; i++) {
        paths.push_back(PathPtr(new KernelPath({0, 1, 0}, i)));
    }
}
#endif

// initialize the policies
void initPolicies(std::vector<PolicyPtr>& policies, uint K, double threshold, double damping_factor)
{
    policies.clear();
    policies.push_back(PolicyPtr(new ConMPTSLatency(K, threshold, damping_factor)));
#if DEBUG_mode
    for (auto p : policies) {
        std::cout << "# Init: " << p->info() << "" << std::endl;
    }
#endif

}

// start simulation
void startSimulation(const uint simulationTimes, const uint T,
        const uint M, const double threshold,
        const std::vector<PathPtr>& paths,
        const std::vector<PolicyPtr>& policies,
        const std::string& logFile,
        const uint delta_t,
        const bool isForever)
{
    uint P = policies.size();
    uint K = paths.size();

    RoundwiseFullLog log(P, T, K, isForever);

    for (uint i = 0; i<simulationTimes; ++i) {
        Simulator<RoundwiseFullLog> pathSelectionSim(paths, policies, M, threshold, delta_t);
        pathSelectionSim.runSimulation(log, T);
    }

    std::cout << "Number of selected paths: " << M << std::endl;
    std::vector<std::string> pathNames;
    pathNames.reserve(paths.size());
    for (const auto& iPath : paths) {
        pathNames.push_back(iPath->printInfo());
    }
    std::vector<std::string> policyNames;
    policyNames.reserve(policies.size());
    for (const auto& iPolicy : policies) {
        policyNames.push_back(iPolicy->name());
    }
    std::cout << "Output reward and violation in: " << logFile << std::endl;
    RoundwiseFullLogWriter::fullLogWrite(log, T, policyNames, logFile);
}

} // name space

