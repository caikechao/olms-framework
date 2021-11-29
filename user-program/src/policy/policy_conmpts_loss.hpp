#pragma once

#include "policy.hpp"
#include "../lpsolver/matrix.h"
#include "../lpsolver/lpsolver.h"

namespace bandit {

//Constrained Multi-Path Thompson sampling (binary reward)
// for the loss aware multi-path selection

class ConMPTSLoss: public Policy {
    const uint K;
    double threshold;
    std::vector<double> sb, fb; //s: success; f: fail
    std::vector<double> sl, fl; //s: success; f: fail
    std::uniform_real_distribution<double> unif;

public:
    ConMPTSLoss(uint K, double threshold, double s = 1, double f = 1)
            :K(K), threshold(threshold), unif(0.0, 1.0)
    {
        for (uint i = 0; i<K; ++i) {
            // for bandwidth
            sb.push_back(s);
            fb.push_back(f);
            // for loss
            sl.push_back(s);
            fl.push_back(f);
        }
    }

    std::vector<uint> selectNextPaths(uint M) override
    {
        // Posterior estimate
        std::vector<double> hatb(K, 0.0);
        std::vector<double> hatl(K, 0.0);
        // Selection vector
        std::vector<double> vt(K, 0.0);
        // Threshold vector

        // Get the selection vector
        for (uint i = 0; i<K; ++i) {
            hatb[i] = beta_distribution<double>(sb[i], fb[i])(randomEngine);
            hatl[i] = beta_distribution<double>(sl[i], fl[i])(randomEngine);
        }
        // Call the LP.
        LPSolver::LPStatus status = solveConTSLP(hatb, hatl, M, threshold, vt);

        if (status==LPSolver::FEASIBLE) {
            printVec("vt in SelectNextPath: ", vt);
            // Select M paths with vector vt
            return dependentRounding(M, vt);
        }
        else {
            return vectorMinIndices(hatl, M);
        }

    }

    // compute the oracle policy according to the bandwidth Ab, and the loss rate Lr
    double computeOracleLoss(
            const std::vector<double>& Ab,
            const std::vector<double>& Lr,
            uint M,
            double threshold)
    {
        std::vector<double> vStar;
        printVar("K", K);
        printVec("Bandwidth", Ab);
        printVec("Loss rate", Lr);
        LPSolver::LPStatus oracleStatus = solveConTSLP(Ab, Lr, M, threshold, vStar);
        if (oracleStatus==LPSolver::ERROR) {
            std::cout << "CONMPTSLoss: cannot find the oracle !!!" << std::endl;
            abort();
        }
        printVec("computeOracleLoss-vStar", vStar);
        return vectorDot(vStar, Ab);
    }

    LPSolver::LPStatus
    solveConTSLP(
            const std::vector<double>& hatb,
            const std::vector<double>& hatl,
            const double Mpath,
            const double Cl, // threshold on the total loss
            std::vector<double>& lp_x)
    {
        /*x = (u, x1, ..., xk)*/
        /* Construct lp_A*/
        Matrix lp_A(2, K);

        // Part 1. for the l'x <= Cl
        for (int i = 0; i<K; ++i) {
            lp_A[0][i] = hatl[i];
        }
        // Part 2. for the 1'v = M
        for (int i = 0; i<K; ++i) {
            lp_A[1][i] = 1;
        }
        /* Construct the standard vector lp_b */
        std::vector<double> lp_b(1, 0);
        lp_b[0] = Cl;
        // Mpath (with last equal constraint) */
        /* Construct the standard vector lp_c */
        const std::vector<double>& lp_c = hatb;
        lp_A.printMat("Loss_lp_A:");
        printVec("lp_b", lp_b);
        printVec("lp_c", lp_c);
        printVar("lp_fix", Mpath);
        LPSolver Solver(lp_A, lp_b, lp_c, Mpath);
        Solver.setMaximization();
        LPSolver::LPStatus status = Solver.solve(lp_x);
//        if (status==LPSolver::ERROR) {
//            // Uniform random
//            for (int j = 0; j<K; ++j) {
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
            double rand_l = unif(randomEngine);

            double tb = bernoulliTrial(rand_b, measurement.b);
            double tl = bernoulliTrial(rand_l, measurement.l);

            if (tb>0.5) { sb[k] += 1; } else { fb[k] += 1; }
            if (tl>0.5) { sl[k] += 1; } else { fl[k] += 1; }

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
        return "ConMPTSLoss";
    }

    std::string info() override
    {
        std::string name = "ConMPTSLoss_aware: ";
        std::string para1 = "K: "+std::to_string(K)+", ";
        std::string para2 = "th: "+std::to_string(threshold)+", ";
        return name+para1+para2;
    }

    PolicyType getType() override
    {
        return PolicyType::CONMPTS_Loss;
    }
};

} //namespace
