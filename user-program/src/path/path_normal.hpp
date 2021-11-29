//
// Created by kccai.
//
#pragma once

#include "../bandit/macro_util.h"
#include "path.hpp"
#include <tuple>

namespace bandit {

typedef std::tuple<double,double> Norm;

class NormalPath: public Path {
    Norm r;
    Norm b;
    Norm l;
    const Metric meanMetric;
    std::uniform_real_distribution<double> unif;
public:
    NormalPath(Norm r_, Norm b_, Norm l_)
            :r(r_), b(b_), l(l_)
    {

    }

    Metric getMeasurement() override
    {

    }

    Metric getMeanMetric() override
    {

    }

    Metric getVarMetric() // specific for normal path
    {

    }

    std::string printInfo() override
    {

    }

    PathType getType() override
    {

    }
};
}

