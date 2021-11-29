#pragma once

#include "../bandit/macro_util.h"
#include "path.hpp"

namespace bandit {

class BernoulliPath: public Path {
    const Metric meanMetric;
    std::uniform_real_distribution<double> unif;

public:
    explicit BernoulliPath(Metric average)
            :meanMetric(Metric{average.r, average.b, average.l}), unif(0.0, 1.0)
    {
    }

    Metric getMeanMetric() override
    {
        return meanMetric;
    }

    Metric getMeasurement() override
    {
        double rand_r = unif(randomEngine);
        double rand_b = unif(randomEngine);
        double rand_l = unif(randomEngine);

        // tr, tb, tl, are either 1.0 or 0.0
        double tr = bernoulliTrial(rand_r, meanMetric.r);
        double tb = bernoulliTrial(rand_b, meanMetric.b);
        double tl = bernoulliTrial(rand_l, meanMetric.l);

        return Metric({tr, tb, tl});
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
        return PathType::BERNOULLI;
    }
};

} //namespace
