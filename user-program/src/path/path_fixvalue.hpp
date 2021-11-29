//
// Created by kccai.
//

#ifndef MULTIPATH_SELECTION_PATH_FIXVALUE_HPP
#define MULTIPATH_SELECTION_PATH_FIXVALUE_HPP

#include "../bandit/macro_util.h"
#include "path.hpp"

namespace bandit {

class FixValuePath: public Path {
    const Metric meanMetric;

public:
    explicit FixValuePath(Metric fix)
            :meanMetric(Metric{fix.r, fix.b, fix.l})
    {
    }

    Metric getMeanMetric() override
    {
        return meanMetric;
    }

    Metric getMeasurement() override
    {
        return Metric({meanMetric.r, meanMetric.b, meanMetric.l});
    }

    std::string printInfo() override
    {
        std::string str =
                "\t"+dtos(meanMetric.r)+
                        "\t"+dtos(meanMetric.b)+
                        "\t"+dtos(meanMetric.l);

        return str;
    }

    PathType getType() override
    {
        return PathType::FIXVALUE;
    }
};

} //namespace

#endif //MULTIPATH_SELECTION_PATH_FIXVALUE_HPP
