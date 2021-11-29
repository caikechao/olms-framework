//
// Created by kccai.
//

#pragma once

#include "matrix.h"

#include <vector>
#include <string>

struct glp_prob;

class LPSolver {
public:
    LPSolver(Matrix& A, const std::vector<double>& b, const std::vector<double>& c, const double&  Fix);

    LPSolver(const LPSolver& p);

    ~LPSolver();

    LPSolver& operator=(const LPSolver& p);

enum LPStatus {
        FEASIBLE,
        ERROR
    };

    virtual LPStatus solve(std::vector<double>& result);

    void setName(const std::string& s);

    bool isValid();

    void setMaximization();

    void setMinimization();

private:
    size_t m_M;
    size_t m_N;
    std::vector<double> m_c;
    std::vector<double> m_b; //
    double m_Fix;  /* FIX equal row bound*/
    Matrix m_A;
    glp_prob* m_lp;

    void initProblem(size_t M, size_t N);

    void setRowBounds();

    void setColumnCoefs();

protected:
    glp_prob* getLP();

    int getNumCols();

    int getNumRows();
};

