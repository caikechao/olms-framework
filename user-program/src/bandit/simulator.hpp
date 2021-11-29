#pragma once

#include "macro_util.h"
#include "bandit_util.hpp"
#include "../path/path.hpp"
#include "../policy/policy.hpp"
#include "../policy/policy_conmpts_latency.hpp"
#include "../policy/policy_conmpts_bandwidth.hpp"
#include "../policy/policy_conmpts_loss.hpp"
#include "../bandit/roundwiselog.hpp"

#include <thread>
#include <chrono>

namespace bandit {

typedef std::shared_ptr<Path> PathPtr;
typedef std::shared_ptr<Policy> PolicyPtr;

template<class Log = RoundwiseLog>
class Simulator {
    // bool recommendBest; // using the best path or M paths
    std::vector<PathPtr> paths;
    std::vector<PolicyPtr> policies;

    const uint M; //selects M out of K paths in each round
    const uint K;
    const double threshold;

    // get the max reward per round of the oracle.
    double oracleRewardAtT;
    // the interval for sleep
    uint delta_t;
    std::vector<uint> all_paths;

public:
    Simulator(const std::vector<PathPtr>& paths, const std::vector<PolicyPtr>& policies, uint M, double threshold,
            uint Delta_t)
            :paths(paths), policies(policies), M(M), K(paths.size()), threshold(threshold), delta_t(Delta_t)
    // recommendBest(theBestpath)
    {
        if (M>K) {
            std::cerr << "Simulator: M > K！ Abort!" << std::endl;
            abort();
        }
        std::vector<double> r;
        std::vector<double> b;
        std::vector<double> l;
        for (const auto& path : paths) {
            Metric mean = path->getMeanMetric();
            r.push_back(mean.r);
            b.push_back(mean.b);
        }
        for (int i = 0; i<paths.size(); ++i) {
            all_paths.push_back(i);
        }
        if (0) {
            // Compute the oracle of the ConMPTSLatency policy
            printMsg("Computing oracle...");
            for (auto& policy: policies) {
                if (policy->getType()==PolicyType::CONMPTS_Latency) {
                    std::shared_ptr<ConMPTSLatency> pConMPTS = std::static_pointer_cast<ConMPTSLatency>(policy);
                    printVec("r", r);
                    printVec("b", b);
                    printVar("M", M);
                    printVar("threshold", threshold);
                    oracleRewardAtT = pConMPTS->computeOracleLatency(r, b, M, threshold);
                }
                if (policy->getType()==PolicyType::CONMPTS_Bandwidth) {
                    std::shared_ptr<ConMPTSBandwidth> pConMPTS = std::static_pointer_cast<ConMPTSBandwidth>(policy);
                    printVec("r", r);
                    printVec("b", b);
                    printVar("M", M);
                    printVar("threshold", threshold);
                    oracleRewardAtT = pConMPTS->computeOracleBandwidth(r, b, M, threshold);
                }
                if (policy->getType()==PolicyType::CONMPTS_Loss) {
                    std::shared_ptr<ConMPTSLoss> pConMPTS = std::static_pointer_cast<ConMPTSLoss>(policy);
                    printVec("r", r);
                    printVec("b", b);
                    printVar("M", M);
                    printVar("threshold", threshold);
                    oracleRewardAtT = pConMPTS->computeOracleLoss(r, b, M, threshold);
                }
            }
        }
    }

    void runSimulation(Log& log, const uint T)
    {
        log.addSimulation();
        // log.oracleRwReward = oracleRewardAtT;
        for (uint t = 0; t<T; ++t) {
            for (uint p = 0; p<policies.size(); ++p) {
                //execSingleRoundDamped(log, p, t);
                execSingleRound(log, p, t);
            }
        }
    }

    void execSingleRound(Log& log, uint p, uint t)
    {
        std::vector<uint> is;
        is = policies[p]->selectNextPaths(M);

#ifdef OLMS_KERNEL
        kolms.setPreferredPaths(is, K);
#endif

#ifdef OLMS_KERNEL
        // the clock is used here.
        // wait for the measurements
        // std::this_thread::sleep_for(std::chrono::milliseconds(1));
        std::this_thread::sleep_for(std::chrono::microseconds(delta_t));
        int kernel_status = kolms.fetchMeasurements();
        //if (kernel_status==-1) {
        if (kernel_status<0 && log.forever) {
            // if (kernel_status<0) {
            std::cout << "status: " << kernel_status << std::endl;
            std::cout << "No measurement. Transmission ended." << std::endl;
            exit(0);
        }
#endif

        std::vector<Metric> measurements;
        // std::vector<double> rewards;
        // std::vector<double> violations;
        Metric measurementAtT;

        for (const auto& i : all_paths) {
            measurementAtT = paths[i]->getMeasurement();

            if (std::find(is.begin(), is.end(), i)!=is.end()) {
                measurements.push_back(measurementAtT);
                log.recordSelectedPaths(p, t, i);
            }
            // rewards.push_back(measurementAtT.b); // get the btlbw measurement as the reward
            // violations.push_back(measurementAtT.r-threshold); // violation of each selected path
            // record full information of this path
            log.recordMeasurements(p, t, i, measurementAtT);
        }

        policies[p]->updateState(is, measurements);

        if (1) { // use log to save the selection vector
            std::cout << "Selected path ID: ";
            for (const auto& i : is) {
                std::cout << i << " ";
            }
            std::cout << std::endl;
            std::cout << std::endl;
        }

        //double rewardAtT = vectorSum(rewards);
        //double regretDeltaAtT = oracleRewardAtT-rewardAtT;
        //double violationAtT = vectorSum(violations);
        //log.record(p, t, rewardAtT, regretDeltaAtT, violationAtT);
    }

    void execSingleRoundDamped(Log& log, uint p, uint t)
    {
        std::vector<uint> is;
        is = policies[p]->selectNextPaths(M);
        std::vector<Metric> measurements;
        std::vector<double> rewards;
        std::vector<double> violations;
        Metric measurementAtT;
        for (const auto& i : is) {
            measurementAtT = paths[i]->getMeasurement();
            measurements.push_back(measurementAtT);
            rewards.push_back(measurementAtT.b); // get the btlbw measurement as the reward
            violations.push_back(measurementAtT.r-threshold); // violation of each selected path
        }
        policies[p]->updateStateDamped(is, measurements);

        if (1) {
            std::cout << "Selected path ID: ";
            for (const auto& i : is) {
                std::cout << i << " ";
            }
            std::cout << std::endl;
        }
        double rewardAtT = vectorSum(rewards);
        double regretDeltaAtT = oracleRewardAtT-rewardAtT;
        double violationAtT = vectorSum(violations);
        log.record(p, t, rewardAtT, regretDeltaAtT, violationAtT);
    }

    void execSingleRoundAvg(Log& log, uint p, uint t)
    {
        std::vector<uint> is;
        is = policies[p]->selectNextPathsAvg(M);
        std::vector<Metric> measurements;
        std::vector<double> rewards;
        std::vector<double> violations;
        Metric measurementAtT;
        for (const auto& i : is) {
            measurementAtT = paths[i]->getMeasurement();
            measurements.push_back(measurementAtT);
            rewards.push_back(measurementAtT.b); // get the btlbw measurement as the reward
            violations.push_back(measurementAtT.r-threshold); // violation of each selected path
        }
        policies[p]->updateStateAvg(is, measurements);

        if (1) {
            std::cout << "Selected path ID: ";
            for (const auto& i : is) {
                std::cout << i << " ";
            }
            std::cout << std::endl;
        }

        double rewardAtT = vectorSum(rewards);
        double regretDeltaAtT = oracleRewardAtT-rewardAtT;
        double violationAtT = vectorSum(violations);
        log.record(p, t, rewardAtT, regretDeltaAtT, violationAtT);
    }

};

} //namespace
