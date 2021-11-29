//
// Created by kccai.
//
#pragma once

// add the macro for debugging locally
#define OLMS_KERNEL 1

#ifndef DEBUG_mode
#define DEBUG_mode 0
#endif

#define LOG_var(x) do { std::cerr <<  "[" << __FILE__ << "][" \
                                << __FUNCTION__ << "][Line " << __LINE__ << "] " \
                                <<#x << ": " << x << std::endl; } while (0)

