#pragma once

#include "policy.hpp"
#include "../lpsolver/matrix.h"
#include "../lpsolver/lpsolver.h"

namespace bandit {

//Constrained Multi-Path Thompson sampling (binary reward)
// for the latency aware multi-path selection

class ConMPTSLatency: public Policy {
    const uint K;
    double threshold;
    double damping_factor;

    std::vector<double> avgb;
    std::vector<double> avgr;
    std::vector<uint> selected_times;
    std::vector<double> sb, fb; //s: success; f: fail
    std::vector<double> sr, fr; //s: success; f: fail
    std::uniform_real_distribution<double> unif;

public:
    ConMPTSLatency(uint K, double threshold, double damping_factor_, double s = 1, double f = 1)
            :K(K), threshold(threshold), damping_factor(damping_factor_), unif(0.0, 1.0)
    {
        for (uint i = 0; i<K; ++i) {
            // for bandwidth
            sb.push_back(s);
            fb.push_back(f);
            // for rtt
            sr.push_back(s);
            fr.push_back(f);
            // average metric init with 0
            avgb.push_back(0.5);
            avgr.push_back(0.5);
            selected_times.push_back(0);
        }
    }

    std::vector<uint> selectNextPaths(uint M) override
    {
        // Posterior estimate
        std::vector<double> hatb(K, 0.0);
        std::vector<double> hatr(K, 0.0);
        // Selection vector
        std::vector<double> vt(K, 0.0);
        // Threshold vector
        std::vector<double> vTh(K, threshold);
        // Get the selection vector
        for (uint i = 0; i<K; ++i) {
            hatb[i] = beta_distribution<double>(sb[i], fb[i])(randomEngine);
            hatr[i] = beta_distribution<double>(sr[i], fr[i])(randomEngine);
            // hatb[i] = sb[i]/(sb[i]+fb[i]);
            // hatr[i] = sr[i]/(sr[i]+fr[i]);
        }

        // printVecR("hatr", hatr);

        // Call the LP.
        LPSolver::LPStatus status = solveConTSLP(hatr, hatb, M, threshold, vt);

        if (status==LPSolver::FEASIBLE) {
            printVec("vt in SelectNextPath: ", vt);
            // Select M paths with vector vt
            // std::cout<< "Feasible "<<std::endl;
            return dependentRounding(M, vt);
        }
        else {
            // std::cout<< "ERROR "<<std::endl;
            return vectorMinIndices(hatr, M);
        }

    }

    // compute the oracle policy according to the bandwidth b, and the rtprop r
    double computeOracleLatency(const std::vector<double>& rtt, const std::vector<double>& bw,
            uint M, double threshold)
    {
        std::vector<double> vStar;
        printVar("K", K);
        printVec("rtt", rtt);
        LPSolver::LPStatus oracleStatus = solveConTSLP(rtt, bw, M, threshold, vStar);
        if (oracleStatus==LPSolver::ERROR) {
            std::cout << "CONMPTSLatency: cannot find the oracle !!!" << std::endl;
            abort();
        }
        printVec("computeOracleLatency-vStar", vStar);
        return vectorDot(vStar, bw);
    }

    LPSolver::LPStatus
    solveConTSLP(
            const std::vector<double>& rtt,
            const std::vector<double>& bw,
            const double Mpath,
            const double Cr,
            std::vector<double>& lp_x)
    {
        // Satisfy the Cr at the best effort
//        std::vector<uint> candidateIdx = vectorLTHIndices(rtt, Cr);
//        if (candidateIdx.size()+0.01>=Mpath) {
//            std::vector<double> candidateBw = newVectorWithIndices(bw, candidateIdx);
//            std::vector<uint> bwIndices = vectorMaxIndices(candidateBw, uint(Mpath));
//            std::vector<uint> pathIndices = newVectorWithIndices(candidateIdx, bwIndices);
//            lp_x.resize(K, 0);
//            for (const auto& i : pathIndices) {
//                lp_x[i] = 1;
//            }
//            printVec("adj-lpx", lp_x);
//            return LPSolver::FEASIBLE;
//        }
//        else {

        // construct A
        Matrix A(K+1, K);
        for (int i = 0; i<K; ++i) {
            A[i][i] = rtt[i];
        }
        // Init the last row
        for (int j = 0; j<K; ++j) {
            A[K][j] = 1;
        }

        // Construct lp_b
        const std::vector<double> lp_b(K, Cr);
        // Construct lp_c
        const std::vector<double>& lp_c = bw;

        A.printMat("Latency_lp_A:");
        printVec("lp_b", lp_b);
        printVec("lp_c", lp_c);
        printVar("lp_fix", Mpath);
        LPSolver Solver(A, lp_b, lp_c, Mpath);
        Solver.setMaximization();
        LPSolver::LPStatus status = Solver.solve(lp_x);
        printVec("lp_x", lp_x);
//        if (status==LPSolver::ERROR) {
//            // Uniform random
//            for (int j = 0; j<K; ++j) {
//                lp_x[j] = (Mpath*1.0)/K;
//            }
//        }
        return status;

//        }

    }

    void updateState(std::vector<uint> selectedPaths, std::vector<Metric> selectedPathMeasurements) override
    {
        for (uint i = 0; i<selectedPaths.size(); ++i) {
            uint k = selectedPaths[i];
            Metric measurement = selectedPathMeasurements[i];

            double rand_b = unif(randomEngine);
            double rand_r = unif(randomEngine);

            double tb = bernoulliTrial(rand_b, measurement.b);
            double tr = bernoulliTrial(rand_r, measurement.r);

            if (tb>0.5) { sb[k] += 1; } else { fb[k] += 1; }
            if (tr>0.5) { sr[k] += 1; } else { fr[k] += 1; }

        }
    }

    void updateStateDamped(std::vector<uint> selectedPaths, std::vector<Metric> selectedPathMeasurements) override
    {
        double memory_factor = 1-damping_factor;
        for (uint j = 0; j<K; ++j) {
            for (uint i = 0; i<selectedPaths.size(); ++i) {
                uint k = selectedPaths[i];
                if (j==k) {
                    Metric measurement = selectedPathMeasurements[i];
                    double rand_b = unif(randomEngine);
                    double rand_r = unif(randomEngine);
                    double tb = bernoulliTrial(rand_b, measurement.b);
                    double tr = bernoulliTrial(rand_r, measurement.r);

                    if (tb>0.5) {
                        sb[j] = memory_factor*(sb[j]+1);
                        sb[j] = max(sb[j], 1.0);
                    }
                    else {
                        fb[j] = memory_factor*(fb[j]+1);
                        fb[j] = max(fb[j], 1.0);
                    }
                    if (tr>0.5) {
                        sr[j] = memory_factor*(sr[j]+1);
                        sr[j] = max(sr[j], 1.0);
                    }
                    else {
                        fr[j] = memory_factor*(fr[j]+1);
                        fr[j] = max(fr[j], 1.0);
                    }
                }
                else { // also forget history of other paths
                    sb[j] = max(memory_factor*sb[j], 1.0);
                    fb[j] = max(memory_factor*fb[j], 1.0);
                    sr[j] = max(memory_factor*sr[j], 1.0);
                    fr[j] = max(memory_factor*fr[j], 1.0);
                }
            }
        }
    }

    std::vector<uint> selectNextPathsAvg(uint M) override
    {

        // Selection vector
        std::vector<double> vt(K, 0.0);
        // Threshold vector
        std::vector<double> vTh(K, threshold);

        printVecR("rtt-avg", avgr);
        // Get the selection vector and Call the LP with the average estimate.
        LPSolver::LPStatus status = solveConTSLP(avgr, avgb, M, threshold, vt);

        if (status==LPSolver::FEASIBLE) {
            printVec("vt in SelectNextPath: ", vt);
            // Select M paths with vector vt
            // std::cout<< "Feasible "<<std::endl;
            return dependentRounding(M, vt);
        }
        else {
            // std::cout<< "ERROR "<<std::endl;
            return vectorMinIndices(avgr, M);
        }
    }

    void updateStateAvg(std::vector<uint> selectedPaths, std::vector<Metric> selectedPathMeasurements) override
    {
        printMsg("Latency-update-avg-start");
        for (uint i = 0; i<selectedPaths.size(); ++i) {
            uint k = selectedPaths[i];
            Metric measurement = selectedPathMeasurements[i];

            uint k_times = selected_times[k];
            double k_avgb = avgb[k];
            double k_avgr = avgr[k];
            // update the average
            avgb[k] = ((k_times*k_avgb)+measurement.b)/(k_times+1.0);
            avgr[k] = ((k_times*k_avgr)+measurement.r)/(k_times+1.0);
            // update the selected times of path k
            selected_times[k] = k_times+1;
        }
        printVec("selected_times", selected_times);
        printMsg("Latency-update-avg-end");
    }

    std::string name() override
    {
        return "ConMPTSLatency";
    }

    std::string info() override
    {
        std::string name = "ConMPTSLatency: ";
        std::string para1 = "K: "+std::to_string(K)+", ";
        std::string para2 = "th: "+std::to_string(threshold)+", ";
        return name+para1+para2;
    }

    PolicyType getType() override
    {
        return PolicyType::CONMPTS_Latency;
    }
};

} //namespace
