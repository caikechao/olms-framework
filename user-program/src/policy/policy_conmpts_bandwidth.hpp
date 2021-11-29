#pragma once

#include "policy.hpp"
#include "../lpsolver/matrix.h"
#include "../lpsolver/lpsolver.h"

namespace bandit {

//Constrained Multi-Path Thompson sampling (binary reward)
// for the bandwidth aware multi-path selection

class ConMPTSBandwidth: public Policy {
    const uint K;
    double threshold;
    std::vector<double> sb, fb; //s: success; f: fail
    std::vector<double> sr, fr; //s: success; f: fail
    std::uniform_real_distribution<double> unif;

public:
    ConMPTSBandwidth(uint K, double threshold, double s = 1, double f = 1)
            :K(K), threshold(threshold), unif(0.0, 1.0)
    {
        for (uint i = 0; i<K; ++i) {
            // for bandwidth
            sb.push_back(s);
            fb.push_back(f);
            // for rtt
            sr.push_back(s);
            fr.push_back(f);
        }
    }

    std::vector<uint> selectNextPaths(uint M) override
    {
        // Posterior estimate
        std::vector<double> hatb(K, 0.0);
        std::vector<double> hatr(K, 0.0);
        // vector lp_x: the size is K+1;
        std::vector<double> lp_x;

        // Get the selection vector
        for (uint i = 0; i<K; ++i) {
            hatb[i] = beta_distribution<double>(sb[i], fb[i])(randomEngine);
            hatr[i] = beta_distribution<double>(sr[i], fr[i])(randomEngine);
        }
        // Call the LP.
        LPSolver::LPStatus status = solveConTSLP(hatr, hatb, M, threshold, lp_x);
        if (status==LPSolver::FEASIBLE) {
            // Selection vector vt: the size is K;
            std::vector<double> vt(lp_x.begin()+1, lp_x.end());
            printVec("vt in SelectNextPath: ", vt);
            // Select M paths with vector vt
            return dependentRounding(M, vt);
        }
        else {
            return vectorMinIndices(hatr, M);
        }
    }

    // compute the oracle policy according to the bandwidth b, and the rtprop r
    double computeOracleBandwidth(
            const std::vector<double>& rtprop,
            const std::vector<double>& btlbw,
            uint M,
            double threshold)
    {
        std::vector<double> vTh(K, threshold);
        std::vector<double> lp_xstar;
        printVar("K", K);
        printVec("vTh", vTh);
        printVec("rtprop", rtprop);
        LPSolver::LPStatus oracleStatus = solveConTSLP(rtprop, btlbw, M, threshold, lp_xstar);
        if (oracleStatus==LPSolver::ERROR) {
            std::cout << "CONMPTSBandwidth: cannot find the oracle !!!" << std::endl;
            abort();
        }
        // std::vector<double> vStar(lp_xstar.begin()+1, lp_xstar.end());
        // printVec("computeOracleLatency-vStar", vStar);
        return lp_xstar[0];
    }

    LPSolver::LPStatus solveConTSLP(
            const std::vector<double>& hatr,
            const std::vector<double>& hatb,
            const double Mpath, // M paths
            const double Cb, // threshold on the total bandwidth
            std::vector<double>& lp_x)
    {
        /*x = (u, x1, ..., xk)*/
        /* construct the standard Matrix A */
        Matrix lp_A(K+2, K+1);
        // Part 1. for the -u + r_i * v_i <= 0
        // first K rows
        for (int i = 0; i<K; ++i) {
            lp_A[i][0] = -1.0;
            // lp_A is the r vector
            lp_A[i][i+1] = hatr[i];
        }
        // Part 2. for the b'*v <= C_b
        // the (K+1)-th row
        lp_A[K][0] = 0;
        for (int i = 1; i<K+1; ++i) {
            lp_A[K][i] = hatb[i];
        }
        // Part 3. Init the (K+2)-th row (equality constraint)
        lp_A[K+1][0] = 0.0;
        for (int j = 1; j<K+1; ++j) {
            lp_A[K][j] = 1;
        }
        /* Construct the standard vector lp_b (with last equal constraint) */
        std::vector<double> lp_b(K+1, 0.0); // for part 1;
        lp_b[K] = Cb; // for Part 2;
        // Mpath is for lp_fix for Part 3;

        /* Construct the standard vector lp_c */
        std::vector<double> lp_c(K+1, 0.0);
        lp_c[0] = 1.0;

        lp_A.printMat("Bandwidth_lp_A:");
        printVec("lp_b", lp_b);
        printVec("lp_c", lp_c);
        printVar("lp_fix", Mpath);
        LPSolver Solver(lp_A, lp_b, lp_c, Mpath);
        // Minimize the maximum rtt
        Solver.setMinimization();
        // Solving the linear program
        LPSolver::LPStatus status = Solver.solve(lp_x);

//        if (status==LPSolver::ERROR) {
//            // Uniform random
//            double max_hatb = vectorMax(hatb);
//            lp_x[0] = Mpath/K*max_hatb;
//            for (int j = 1; j<K+1; ++j) {
//                lp_x[j] = (Mpath*1.0)/K;
//            }
//        }
        return status;
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

    std::vector<uint> selectNextPathsAvg(uint M) override
    {

    }

    void updateStateAvg(std::vector<uint> selectedPaths, std::vector<Metric> selectedPathMeasurements) override
    {

    }

    std::string name() override
    {
        return "ConMPTSBandwidth_aware";
    }

    std::string info() override
    {
        std::string name = "ConMPTSBandwidth_aware: ";
        std::string para1 = "K: "+std::to_string(K)+", ";
        std::string para2 = "th: "+std::to_string(threshold)+", ";
        return name+para1+para2;
    }

    PolicyType getType() override
    {
        return PolicyType::CONMPTS_Bandwidth;
    }
};

} //namespace
