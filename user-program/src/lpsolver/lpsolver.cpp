//
// Created by kccai.
//


#include "../bandit/macro_util.h"
#include "lpsolver.h"

#include <glpk.h>
#include <iostream>

using std::vector;
using std::string;
using std::cout;
using std::endl;

LPSolver::LPSolver(Matrix& m, const vector<double>& b, const vector<double>& c, const double& Fix)
        :m_M(m.numRows()),
         m_N(m[0].size()),
         m_c(c),
         m_b(b),
         m_A(m),
         m_Fix(Fix),
         m_lp(glp_create_prob())
{
    initProblem(m_M, m_N);
}

LPSolver::LPSolver(const LPSolver& p)
        :m_M(p.m_M),
         m_N(p.m_N),
         m_c(p.m_c),
         m_b(p.m_b),
         m_A(p.m_A),
         m_Fix(p.m_Fix),
         m_lp(glp_create_prob())
{
    initProblem(m_M, m_N);
}

// performs necessary initialization of the given values
void LPSolver::initProblem(size_t M, size_t N)
{
    if (!m_lp) return;

    setRowBounds();
    setColumnCoefs();

    vector<int> I, J;
    vector<double> V;

    // indices in GLPK start on 1
    I.push_back(0);
    J.push_back(0);
    V.push_back(0);
    for (int i = 0; i<M; ++i) {
        for (int j = 0; j<N; ++j) {
            I.push_back(i+1);
            J.push_back(j+1);
            V.push_back(m_A[i][j]);
        }
    }
    glp_load_matrix(m_lp, (int) (m_M*m_N), &I[0], &J[0], &V[0]);
}

LPSolver::~LPSolver()
{
    glp_delete_prob(m_lp);
}

LPSolver& LPSolver::operator=(const LPSolver& p)
{
    if (this!=&p) {
        m_M = p.m_M;
        m_N = p.m_N;
        m_c = p.m_c;
        m_b = p.m_b;
        m_A = p.m_A;
        m_lp = glp_create_prob();
        initProblem(m_M, m_N);
    }
    return *this;
}

void LPSolver::setName(const std::string& s)
{
    glp_set_prob_name(m_lp, s.c_str());
}

bool LPSolver::isValid()
{
    return m_lp!=NULL;
}

void LPSolver::setMaximization()
{
    glp_set_obj_dir(m_lp, GLP_MAX);
}

void LPSolver::setMinimization()
{
    glp_set_obj_dir(m_lp, GLP_MIN);
}

// this function is important !!!
void LPSolver::setRowBounds()
{
    glp_add_rows(m_lp, (int) m_M);
    for (int i = 0; i<m_M-1; ++i) {
        glp_set_row_bnds(m_lp, i+1, GLP_UP, 0.0, m_b[i]);
    }
    // Only consider a single equality constraint!
    glp_set_row_bnds(m_lp, (int) m_M, GLP_FX, m_Fix, m_Fix); // last Fix constraint
}

// v_i should be in [0, 1]
void LPSolver::setColumnCoefs()
{
    glp_add_cols(m_lp, (int) m_N);
    for (int j = 0; j<m_N; ++j) {
        // double side constraint for each v_i
        glp_set_col_bnds(m_lp, j+1, GLP_DB, 0.0, 1.0);
        glp_set_obj_coef(m_lp, j+1, m_c[j]);
    }
}

LPSolver::LPStatus LPSolver::solve(std::vector<double>& result)
{

    // m_A.printMat("LP-A:");
    // set the LP parameters
    glp_smcp parm;
    glp_init_smcp(&parm);
    parm.msg_lev = GLP_MSG_ERR;
    parm.meth = GLP_DUALP;
    int LP_ENUM = glp_simplex(m_lp, &parm);
    if (LP_ENUM==0) {
        // resized here!!!
        result.resize(m_N, 0);
        // objValue = glp_get_obj_val(m_lp);
        for (int j = 0; j<m_N; ++j) {
            result[j] = glp_get_col_prim(m_lp, j+1);
        }

        return LPSolver::FEASIBLE;

    }
    else {

#if DEBUG_mode  /// debug print
        //        cout<<"LPStatus: " << LP_ENUM <<endl;
#endif
        return LPSolver::ERROR;
    }

}

glp_prob* LPSolver::getLP()
{
    return m_lp;
}

int LPSolver::getNumCols()
{
    return (int) m_N;
}

int LPSolver::getNumRows()
{
    return (int) m_M;
}


