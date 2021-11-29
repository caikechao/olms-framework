#pragma once

#include "../bandit/macro_util.h"
#include "../bandit/kernel_util.hpp"
#include "path.hpp"

namespace bandit {

class KernelPath: public Path {
    const Metric meanMetric;
    std::uniform_real_distribution<double> unif;
    int path_idx;

public:
    explicit KernelPath(Metric average, int idx)
            :meanMetric(Metric{average.r, average.b, average.l}), unif(0.0, 1.0),
             path_idx(idx)
    {
    }

    Metric getMeanMetric() override { return meanMetric; }

    Metric getMeasurement() override
    {
        // double rand_r = unif(randomEngine);
        // double rand_b = unif(randomEngine);
        // double rand_l = unif(randomEngine);

        // tr, tb, tl, are either 1.0 or 0.0
        double tr = kolms.rvec[path_idx];
        double tb = kolms.bvec[path_idx];
        double tl = kolms.lvec[path_idx];

        // double tb = bernoulliTrial(rand_b, meanMetric.b);
        // double tl = bernoulliTrial(rand_l, meanMetric.l);
        //  if (1) {
        //      std::cout << "play[" << path_idx << "]: " << tr << std::endl;
        //      std::cout << std::endl;
        //  }

        return Metric({tr, tb, tl});
    }

    std::string printInfo() override
    {
        std::string str = "\t"+dtos(meanMetric.r)+"\t"+
                dtos(meanMetric.b)+"\t"+dtos(meanMetric.l);

        return str;
    }

    PathType getType() override { return PathType::KERNEL; }
};

} // namespace bandit
