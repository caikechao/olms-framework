#pragma once

#include "policy.hpp"

namespace bandit {

class KLUCBPolicy: public Policy {
    const uint K;
    std::vector<int> Ni;
    std::vector<double> Gi;
    const double DELTA;
    const double EPS;
    bool basic;

    double kl(double p, double q)
    { //calculate kl-divergence
        const double v = p*log(p/q)+(1-p)*log((1-p)/(1-q));
        return v;
    }

    double dkl(double p, double q)
    {
        return (q-p)/(q*(1.0-q));
    }

public:
    KLUCBPolicy(int K, bool basic = true)
            :K(K), DELTA(1e-8), EPS(1e-12), basic(basic)
    {
        reset();
    }

    void reset()
    {
        Ni = std::vector<int>(K, 0);
        Gi = std::vector<double>(K, 0.0);
    }

    double getKLUCBUpper(int k, int n)
    {
        const double logndn = log(n)/(double) Ni[k];
        const double p = std::max(Gi[k]/(double) Ni[k], DELTA);
        if (p>=1) return 1;
        bool converged = false;
        double q = p+DELTA;
        for (int t = 0; (t<20 && !converged); ++t) {
            const double f = logndn-kl(p, q);
            const double df = -dkl(p, q);
            if (f*f<EPS) {
                converged = true;
                break;
            }
            q = std::min(1-DELTA, std::max(q-f/df, p+DELTA));
        }
        if (!converged) {
            //std::cout << "WARNING:Newton iteration in KL-UCB policy did not converge!! p=" << p << " logndn=" << logndn  << std::endl;
        }
        return q;
    }

    std::vector<uint> selectNextPaths(uint l) override
    {
        const double n = vectorSum(Ni);
        if (basic) {
            std::vector<double> indices = std::vector<double>(K, 0.0);
            for (uint k = 0; k<K; ++k) {
                //KL-UCB index
                if (Ni[k]==0) {
                    indices[k] = 100000000; //very large number
                }
                else {
                    indices[k] = getKLUCBUpper(k, n);
                }
            }
            std::vector<uint> targetPaths = vectorMaxIndices(indices, l);
            return targetPaths;
        }
        else {
            std::vector<double> means(K, 0.0);
            for (uint i = 0; i<K; ++i) {
                means[i] = (Gi[i]+1)/(Ni[i]+1);
            }
            std::vector<uint> meanIndices = vectorMaxIndices(means, l); //l-1 ko wo means de
            std::map<uint, bool> meanIndicesMap;
            for (uint i = 0; i<l-1; ++i) { //l-1 <- notice
                meanIndicesMap[meanIndices[i]] = true;
            }

            std::vector<double> ucbIndices = std::vector<double>(K, 0.0);
            for (uint k = 0; k<K; ++k) {
                //KL-UCB index
                if (Ni[k]==0) {
                    ucbIndices[k] = 100000000; //very large number
                }
                else {
                    ucbIndices[k] = getKLUCBUpper(k, n);
                }
            }
            std::vector<uint> ucbPaths = vectorMaxIndices(ucbIndices, l);

            for (uint i = 0; i<l; ++i) {
                uint index = ucbPaths[i];
                if (meanIndicesMap.find(index)==meanIndicesMap.end()) {
                    meanIndices[l-1] = index;
                    break;
                }
            }
            return meanIndices;
        }

    }


    void updateState(std::vector<uint> is, std::vector<Metric> rs) override
    {
        for (uint i = 0; i<is.size(); ++i) {
            uint k = is[i];
            Ni[k] += 1;
            Gi[k] += rs[i].b; // using btlbw to adjust the gain
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
        return "KL-UCB";
    }

    std::string info() override
    {
        std::string str = "KLUCB:";
        return str;
    }

    PolicyType getType() override
    {
        return PolicyType::KLUCB;
    }
};

} //namespace
