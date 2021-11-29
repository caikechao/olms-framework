#pragma once

#include "../bandit/bandit_util.hpp"
#include "../bandit/distributions.hpp"

namespace bandit {

enum PolicyType {
    CONMPTS_Latency,
    CONMPTS_Loss,
    CONMPTS_Bandwidth,
    MPTS,
    KLUCB,
    EXP3M,
    RANDOM
};

class Policy {
public:
    virtual std::vector<uint> selectNextPaths(uint) = 0;

    virtual void updateState(std::vector<uint>, std::vector<Metric>) = 0;

    virtual void updateStateDamped(std::vector<uint>, std::vector<Metric>) = 0;

    virtual std::vector<uint> selectNextPathsAvg(uint M) = 0;

    virtual void updateStateAvg(std::vector<uint>, std::vector<Metric>) = 0;

    virtual std::string name() = 0;

    virtual std::string info() = 0;

    virtual PolicyType getType() = 0;

};

} //namespace
