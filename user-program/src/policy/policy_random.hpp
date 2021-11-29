#pragma once

#include "policy.hpp"

namespace bandit {

class RandomPolicy: public Policy {
    const uint K;
public:
    RandomPolicy(uint K)
            :K(K)
    {
    }

    std::vector<uint> selectNextPaths(uint l) override
    {
        std::map<uint, bool> pathsMap;
        while (pathsMap.size()<l) {
            pathsMap[std::uniform_int_distribution<int>(0, K-1)(randomEngine)] = true;
        }
        std::vector<uint> paths;
        for (const auto& it: pathsMap) {
            paths.push_back(it.first);
        }
        return paths;
    }


    void updateState(std::vector<uint>, std::vector<Metric>) override
    {
        // do not remember anything
    }


    std::vector<uint> selectNextPathsAvg(uint M) override
    {

    }

    void updateStateAvg(std::vector<uint> selectedPaths, std::vector<Metric> selectedPathMeasurements) override
    {

    }

    std::string name() override
    {
        return "RANDOM";
    }

    std::string info() override
    {
        std::string str = "Random:";
        return str;
    }

    PolicyType getType() override
    {
        return PolicyType::RANDOM;
    }
};

} //namespace
