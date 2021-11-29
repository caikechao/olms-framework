#pragma once
#include "macro_util.h"
#include <algorithm>
#include <array>
#include <cfloat>
#include <cmath>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <memory>
#include <queue>
#include <random>
#include <set>
#include <sstream>
#include <string>
#include <vector>

#include <fcntl.h>
#include <sys/ioctl.h>

#define CONSTANT_BOUND 1

extern "C" {
#include "olms-helper.h"
}

namespace bandit {

const int MAX_NUM_PATHS = 64;

struct OLMSKernel {
    std::vector<double> rvec;
    std::vector<double> bvec;
    std::vector<double> lvec;
    uint num_paths;
    uint max_rtt;
    uint max_btlbw;
    int fd;

    OLMSKernel(void)
            :rvec(MAX_NUM_PATHS, 0), bvec(MAX_NUM_PATHS, 0),
             lvec(MAX_NUM_PATHS, 0), max_rtt(0), fd(0)
    {
        fd = open("/dev/olms", O_RDWR);
        if (fd==-1) {
            std::cerr << "Failed to open kernel interface" << std::endl;
            exit(EXIT_FAILURE);
        }
    }

    void set_num_paths(uint v) { num_paths = v; }
    void set_max_rtt(uint v) { max_rtt = v; }
    void set_max_btlbw(uint v) { max_btlbw = v; }

    unsigned long getNumPaths(void)
    {
        struct olms_cmd_args args = {0};
        int ret;

        ret = ioctl(fd, OLMS_IOC_NUMPATHS, &args);
        if (ret==-1) {
            // std::cerr << "ioctl NUMPATHS failed" << std::endl;
            return 0;
        }
        return args.len;
    }

    int fetchMeasurements(void)
    {
        struct olms_cmd_args args = {0};
        std::vector<uint>* rtt_vec, * bw_vec;

        // get the lost packets and delivered packets
        std::vector<uint>* loss_vec, * delivered_vec;

        std::vector<double>* rtt_vec_float, * bw_vec_float;
        std::vector<double>* loss_vec_float;
        int ret;
        unsigned long num_paths;
        uint big_rtt = 0;

        if ((num_paths = getNumPaths())==0)
            return -2;

        // Pre-allocate, as the kernel will access it directly
        rtt_vec = new std::vector<uint>(MAX_NUM_PATHS);
        bw_vec = new std::vector<uint>(MAX_NUM_PATHS);
        // loss and deliver
        loss_vec = new std::vector<uint>(MAX_NUM_PATHS);
        delivered_vec = new std::vector<uint>(MAX_NUM_PATHS);

        args.rtt_vec_addr = (unsigned long) rtt_vec->data();
        args.bw_vec_addr = (unsigned long) bw_vec->data();

        args.loss_vec_addr = (unsigned long) loss_vec->data();
        // args.delivered_vec_addr = (unsigned long) delivered_vec->data();

        retry:
        ret = ioctl(fd, OLMS_IOC_GET_RTT, &args);
        if (ret<0) {
            std::cout << "ioctl GET_RTT failed" << std::endl;
            return ret;
        }

        // Don't pre-allocate, because later I use push_back
        rtt_vec_float = new std::vector<double>;
        bw_vec_float = new std::vector<double>;
        loss_vec_float = new std::vector<double>;

        if (!CONSTANT_BOUND) {
            for (const uint& rtt : *rtt_vec) {
                if (rtt>big_rtt)
                    big_rtt = rtt;
            }
            if (big_rtt>max_rtt)
                max_rtt = big_rtt;
        }

        // ensure at least X paths, and mark unused ones as 0
#if 1
        for (int i = 0; i<args.len; i++) {
            uint rtt = rtt_vec->at(i);
            if (rtt==0) {
                continue;
            }
            // double rtt_float = (double) rtt/(8.0*max_rtt); // already divide by 8 here!
            double rtt_float = (double) rtt/(max_rtt); // restored the rtt in kernel
            if (rtt_float>1.0) {
                rtt_float = 1.0;
            }
            rtt_vec_float->push_back(rtt_float);
            bw_vec_float->push_back((double) bw_vec->at(i)/max_btlbw);
            uint delivered_n = delivered_vec->at(i);
            uint loss_n = loss_vec->at(i);
            if (delivered_n==0) { // the denominator is 0 ...
                loss_vec_float->push_back(0.0);
            }
            else { // the rtt is valid, and the denominator is not 0
                loss_vec_float->push_back((1.0*loss_n)/(1.0*delivered_n));
            }

        }
#else
        rtt_vec_float->push_back(rand()%2);
        rtt_vec_float->push_back(0.3);
        rtt_vec_float->push_back(0.3);
        rtt_vec_float->push_back(rand()%2);
#endif
        if (rtt_vec_float->size()<num_paths) {
            /* retry */
            delete rtt_vec_float;
            delete bw_vec_float;
            delete loss_vec_float;
            goto retry;
        }

        rvec.swap(*rtt_vec_float);
        bvec.swap(*bw_vec_float);
        lvec.swap(*loss_vec_float);

        if (1 /* DEBUG_mode */) {
            std::cout << "#rtt-raw: ";
            for (int i = 0; i<args.len; i++) {
                std::cout << rtt_vec->at(i) << ' ';
            }
            std::cout << std::endl;
            std::cout << "#rtt-rel: ";
            for (const auto& r : rvec) {
                std::cout << r << ' ';
            }
            std::cout << std::endl;
            std::cout << "#bw-raw: ";
            for (int i = 0; i<args.len; i++) {
                std::cout << bw_vec->at(i) << ' ';
            }
            std::cout << std::endl;
            std::cout << "#bw-rel: ";
            for (const auto& b : bvec) {
                std::cout << b << ' ';
            }
            std::cout << std::endl;
            std::cout << "#lossrate: ";
            for (const auto& l : lvec) {
                std::cout << l << ' ';
            }
            std::cout << std::endl;
            // std::cout << std::endl;
        }

        delete rtt_vec;
        delete bw_vec;

        delete loss_vec;
        delete delivered_vec;
        delete loss_vec_float;

        delete rtt_vec_float;
        delete bw_vec_float;

        return 0;
    }

    int setPreferredPaths(std::vector<uint>& is, uint K)
    {
        // Fix the path mapping
        std::vector<uint> kernel_path_ids;
        for (const auto i : is) {
            kernel_path_ids.push_back((K+1)*i); // kernel has already plus 1
        }
        // Feed the data to the kernel
        struct olms_cmd_args args{
                (unsigned long) kernel_path_ids.data(), 0, kernel_path_ids.size(),
        };
        int ret;

        ret = ioctl(fd, OLMS_IOC_PREFER, &args);
        if (ret<0) {
            std::cerr << "ioctl PREFER failed" << std::endl;
            return ret;
        }
        return args.len;
    }
} kolms;
} // namespace bandit
