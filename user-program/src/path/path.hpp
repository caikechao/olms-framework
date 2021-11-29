#pragma once

#include "../bandit/macro_util.h"
#include "../bandit/bandit_util.hpp"

namespace bandit {

enum PathType {
    FIXVALUE,
    BERNOULLI,
    NORMAL,
    KERNEL
};

// Path base class
class Path {
public:
    //base functions should not be called
    virtual Metric getMeasurement() = 0;

    virtual Metric getMeanMetric() = 0;

    virtual std::string printInfo() = 0;

    virtual PathType getType() = 0;
};
} //namespace
