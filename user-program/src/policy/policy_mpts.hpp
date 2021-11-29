#pragma once

#include "policy.hpp"

namespace bandit {

//Unconstrained Multi-played Thompson sampling (binary reward)
class MPTS: public Policy {
    const uint K;
    std::vector<double> alphas, betas;
public:
    MPTS(uint K, bool basic = true, double alpha = 1, double beta = 1)
            :K(K)
    {
        for (uint i = 0; i<K; ++i) {
            alphas.push_back(alpha);
            betas.push_back(beta);
        }
    }

    std::vector<uint> selectNextPaths(uint m) override
    {
        std::vector<double> thetas(K, 0.0);
        for (uint i = 0; i<K; ++i) {
            thetas[i] = beta_distribution<double>(alphas[i], betas[i])(randomEngine);
        }
        std::vector<uint> thetaIndices = vectorMaxIndices(thetas, m);

        return thetaIndices;
    }

    void updateState(std::vector<uint> is, std::vector<Metric> rs) override
    {
        for (uint i = 0; i<is.size(); ++i) {
            uint k = is[i];
            double b = rs[i].b;
            if (b>0.5) {
                alphas[k] += 1;
            }
            else {
                betas[k] += 1;
            }
        }
    }

    std::vector<uint> selectNextPathsAvg(uint M) override
    {

    }

    void updateStateAvg(std::vector<uint> selectedPaths, std::vector<Metric> selectedPathMeasurements) override
    {

    }

    std::string name() override
    {
        return "MP-TS";
    }

    std::string info() override
    {
        std::string str = "MPTS:";
        return str;
    }

    PolicyType getType() override
    {
        return PolicyType::MPTS;
    }
};

} //namespace
