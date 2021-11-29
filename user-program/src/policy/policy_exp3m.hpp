#pragma once

#include "policy.hpp"

namespace bandit {

class Exp3MPolicy: public Policy {
    // always init alpha to -1.0
    double alpha_t = -1.0;
    const uint K;
    const double gamma;
    std::vector<double> wi;
    std::vector<double> pi;
public:
    Exp3MPolicy(uint K, double gamma)
            :K(K), gamma(gamma)
    {
        reset();
    }

    void reset()
    {
        wi = std::vector<double>(K, 1.0);
        pi = std::vector<double>(K, 1.0); //initial value of pi should not be used
    }

    double getAlpha(double rhs, const std::vector<double>& wsSorted)
    {
        double s = vectorSum(wsSorted);
        for (uint k = 0; k<K; ++k) {
            double alpha = (rhs*s)/(1-k*rhs);
            double current = wsSorted[k];
            if (alpha>current) {
                return alpha;
            }
            s -= current;
        }
        std::cerr << "Error: alpha not found." << std::endl;
        abort();
    }

    std::vector<uint> selectNextPaths(uint l) override
    {
        const double wsum = vectorSum(wi);
        std::vector<double> wsSorted(wi);
        std::sort(wsSorted.begin(), wsSorted.end(), std::greater<double>());
        double threshold = (1.0/l-gamma/K)*wsum/(1-gamma);
        std::vector<double> wsd(wi);
        if (wsSorted[0]>=threshold) {
            alpha_t = getAlpha((1.0/l-gamma/K)/(1-gamma), wsSorted);
            for (uint i = 0; i<K; ++i) {
                wsd[i] = std::min(alpha_t, wi[i]);
            }
        }
        else {
            alpha_t = -1.0; // the set S0 is empty
            for (uint i = 0; i<K; ++i) {
                wsd[i] = wi[i];
            }
        }
        const double wsdsum = vectorSum(wsd);
        for (uint i = 0; i<K; ++i) {
            pi[i] = l*((1-gamma)*wsd[i]/wsdsum+gamma/K);
        }
        return dependentRounding(l, pi);
    }

    void updateState(std::vector<uint> is, std::vector<Metric> rs) override
    {
        const uint l = is.size();
        for (uint i = 0; i<is.size(); ++i) {
            uint k = is[i];
            Metric measurement = rs[i];

            if (wi[k]<alpha_t)
                // using btlbw to adjust the weights
                wi[k] *= exp(l*gamma*(measurement.b/pi[k])/K);
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
        return "EXP3M";
    }

    std::string info() override
    {
        std::string str = "Exp3M with gamma=";
        str += dtos(gamma);
        return str;
    }


    PolicyType getType() override
    {
        return PolicyType::EXP3M;
    }
};

} //namespace
