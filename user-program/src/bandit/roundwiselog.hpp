#pragma once

#include "macro_util.h"
#include "bandit_util.hpp"
#include "../path/path.hpp"
#include "../policy/policy.hpp"

#define NUM_PRECISION 4

namespace bandit {

class RoundwiseLog {
public:

    uint P, T, simulationTimes;

    // roundwise reward is the total bandwidth at time t
    vec2Double roundwiseRewards;

    // roundwise violation is the sum(r - h) at time t
    vec2Double roundwiseViolations;

    // roundwise regret is the loss compared to the Oracle
    vec2Double roundwiseRegrets;

    // roundwise measurement is the measurement at time t
    // vec2Metric roundwiseMeasurements;

    // RoundwiseLog(uint K, uint P, uint T,
    //        const std::vector<double>&& optimalPolicy)
    //        :K(K), P(P), T(T), vStar(optimalPolicy)
    double oracleRwReward;

    RoundwiseLog(uint P, uint T)
            :P(P), T(T)
    {
        roundwiseRewards = vec2Double(P, std::vector<double>(T, 0.0));
        roundwiseRegrets = vec2Double(P, std::vector<double>(T, 0.0));
        roundwiseViolations = vec2Double(P, std::vector<double>(T, 0.0));
        // roundwiseMeasurements = vec2Metric(P, std::vector<Metric>(T, {0.0, 0.0, 0.0}));
        simulationTimes = 0; // times of simulation
        oracleRwReward = 0.0;
    }

    //start new simulation
    void addSimulation()
    {
        simulationTimes += 1;
    }

    // policy p at round t, chose a set of paths, received reward r,
    // incurred the regretDelta and the violation
    void record(uint p, uint t, double rewardAtT, double regretDeltaAtT, double violationAtT)
    {
        roundwiseRewards[p][t] += rewardAtT;
        roundwiseRegrets[p][t] += regretDeltaAtT;
        roundwiseViolations[p][t] += violationAtT; // sum (r-h), this can be negative
    }
};

class RoundwiseLogWriter {
public:
    static void logWrite(RoundwiseLog& log, uint T, const std::vector<std::string>& policyNames,
            const std::string& outputFile)
    {
        // const uint K = pathNames.size();
        const uint P = policyNames.size();
        std::ofstream ofs(outputFile);
        ofs << "#averaged result over " << log.simulationTimes
            << " simulations." << std::endl;
        for (uint p = 0; p<P; ++p) {
            ofs << "#policy " << p << " " << policyNames[p] << std::endl;
        }
        // for (uint i = 0; i<K; ++i) {
        //        ofs << "#path" << i << " " << pathNames[i] << std::endl;
        // }
#if DEBUG_mode
        for (uint p = 0; p<P; ++p) {
            std::cout << "#policy " << p << " " << policyNames[p] << std::endl;
        }
#endif
        ofs.setf(std::ios::fixed, std::ios::floatfield);

        // write the header
        ofs << "#results:" << std::endl;
        ofs << "#T " << "Oracle";
        for (uint p = 0; p<P; ++p) {
            ofs << " reward(" << policyNames[p] << ")";
        }
        for (uint p = 0; p<P; ++p) {
            ofs << " regret(" << policyNames[p] << ")";
        }
        for (uint p = 0; p<P; ++p) {
            ofs << " violation(" << policyNames[p] << ")";
        }
        ofs << std::endl;

#if DEBUG_mode
        std::cout << "#results:" << std::endl;
        std::cout << "#T";
        for (uint p = 0; p<P; ++p) {
            std::cout << " reward(" << policyNames[p] << ")";
        }
        for (uint p = 0; p<P; ++p) {
            std::cout << " regret(" << policyNames[p] << ")";
        }
        for (uint p = 0; p<P; ++p) {
            std::cout << " violation(" << policyNames[p] << ")";
        }
        std::cout << std::endl;
#endif

        // write the reward, regret, and violation of each policy
        std::vector<double> cumReward(P, 0.0);
        std::vector<double> cumRegret(P, 0.0);
        std::vector<double> cumViolation(P, 0.0);

        std::cout.precision(NUM_PRECISION);
        for (uint t = 0; t<T; ++t) {
            ofs << (t+1);
            for (uint p = 0; p<P; ++p) {
                cumReward[p] += log.roundwiseRewards[p][t];
                cumRegret[p] += log.roundwiseRegrets[p][t];
                cumViolation[p] += log.roundwiseViolations[p][t];
                ofs << " " << (t*log.oracleRwReward)/log.simulationTimes; // oracle
                ofs << " " << cumReward[p]/log.simulationTimes; // reward
                ofs << " " << cumRegret[p]/log.simulationTimes; // regret
                ofs << " " << max(cumViolation[p]/log.simulationTimes, 0.0); //violation
            }
            ofs << std::endl;
        }
    }
};

class RoundwiseFullLog {
public:

    uint P, T, K, simulationTimes;
    bool forever;

    // vector<vector<vector<string>>> vec1(DIM1, vector<vector<string>>(DIM2, vector<string>(DIM3)));
    vec3Metric policyTimeKMetric;
    vec3Uint policyTimeKpaths;

    RoundwiseFullLog(uint P, uint T, uint K, bool isForever_)
            :P(P), T(T), K(K), forever(isForever_)
    {
        if (!forever) {
            policyTimeKMetric = vec3Metric(P,
                    std::vector<std::vector<Metric> >(T, std::vector<Metric>(K, {0.0, 0.0, 0.0})));
            policyTimeKpaths = vec3Uint(P,
                    std::vector<std::vector<uint> >(T, std::vector<uint>(K, 0)));
            simulationTimes = 0; // times of simulation
        }
    }

    //start new simulation
    void addSimulation()
    {
        if (!forever) {
            simulationTimes += 1;
        }
    }

    // policy p at round t, get path i with measurment m,
    // policy p at rount t, select path i
    void recordMeasurements(uint p, uint t, uint i, Metric m)
    {
        if (!forever) {
            policyTimeKMetric[p][t][i] += m;
        }
    }

    void recordSelectedPaths(uint p, uint t, uint i)
    {
        if (!forever) {
            policyTimeKpaths[p][t][i] += 1;
        }
    }
};

class RoundwiseFullLogWriter {
public:
    static void fullLogWrite(RoundwiseFullLog& fulllog, uint T, const std::vector<std::string>& policyNames,
            const std::string& outputFile)
    {
        if (!fulllog.forever) {
            const uint P = policyNames.size();
            std::ofstream ofs(outputFile);
            ofs << "# result in: " << fulllog.simulationTimes
                << " simulations." << std::endl;
            for (uint p = 0; p<P; ++p) {
                ofs << "# policy " << p << " " << policyNames[p] << std::endl;
            }
            ofs.setf(std::ios::fixed, std::ios::floatfield);

            // write the header
            ofs << "#results:" << std::endl;
            ofs << "#T ";
            for (uint p = 0; p<P; ++p) {
                ofs << " selection(" << policyNames[p] << ")";
            }
            for (uint p = 0; p<P; ++p) {
                ofs << " rtt(" << policyNames[p] << ")";
            }
            for (uint p = 0; p<P; ++p) {
                ofs << " bw(" << policyNames[p] << ")";
            }
            for (uint p = 0; p<P; ++p) {
                ofs << " lossrate(" << policyNames[p] << ")";
            }
            ofs << std::endl;

            // write the selection, rtt, bw and lossrate of each policy
            std::cout.precision(NUM_PRECISION);
            for (uint t = 0; t<T; ++t) {
                ofs << (t+1);
                for (uint p = 0; p<P; ++p) {
#if DEBUG_mode
                    // print the selected times
                    std::cout << "#t: " << t;
                    for (const auto& i : fulllog.policyTimeKpaths[p][t]) {
                        std::cout << " " << (i*1.0)/fulllog.simulationTimes;
                    }
                    std::cout << std::endl;
                    // print the rtt measurement
                    std::cout << "#t: " << t;
                    for (const auto& m : fulllog.policyTimeKMetric[p][t]) {
                        std::cout << " " << (m.r)/fulllog.simulationTimes;
                    }
                    std::cout << std::endl;
#endif
                    // log the selection vector
                    for (const auto& i : fulllog.policyTimeKpaths[p][t]) {
                        ofs << " " << (i*1.0)/fulllog.simulationTimes;
                    }
                    // log the rtt measurements
                    for (const auto& m : fulllog.policyTimeKMetric[p][t]) {
                        ofs << " " << (m.r)/fulllog.simulationTimes;
                    }
                    // log the bw measurements
                    for (const auto& m : fulllog.policyTimeKMetric[p][t]) {
                        ofs << " " << (m.b)/fulllog.simulationTimes;
                    }
                    // log the loss rate measurements
                    for (const auto& m : fulllog.policyTimeKMetric[p][t]) {
                        ofs << " " << (m.l)/fulllog.simulationTimes;
                    }
                }
                ofs << std::endl;
            }
        }
    }
};

} //namespace
