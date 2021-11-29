cmake_minimum_required(VERSION 3.3)
project(multipath-selection)

#like -Wall -Werror -std=c++0x you’ll quickly notice that multiple append will
#yield semicolon separate values. this is actually not what you want.
#
#instead you could use set(CMAKE_CXX_FLAGS “-Wall -O2”) which in turn
#overrides every previous value.
#
#set(CMAKE_CXX_FLAGS “-O2”) ## Optimize
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")
#
#This sets the value to -O2 and then appends -Wall.

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -DDEBUG_mode=1")

#set(CMAKE_BUILD_TYPE Release)
#
### Compiler flags
if (CMAKE_COMPILER_IS_GNUCXX)
    set(CMAKE_CXX_FLAGS "-O3")        ## Optimize
    set(CMAKE_EXE_LINKER_FLAGS "-s")  ## Strip binary
endif ()

find_path(GLPK_INCLUDE_DIR
        NAMES glpk.h
        PATHS ${GLPK_PREFIX} ${GLPK_ROOT}
        PATH_SUFFIXES "glpk" "include/glpk" "include" "SRC" "src"
        NO_DEFAULT_PATH)

# look for header files, including default paths
find_path(GLPK_INCLUDE_DIR
        NAMES glpk.h
        PATH_SUFFIXES "glpk" "include/glpk" "include" "SRC" "src")

# look for library, only at positions given by the user
find_library(GLPK_LIBRARY
        NAMES "glpk"
        PATHS ${GLPK_PREFIX} ${GLPK_ROOT} ${GLPK_ROOT}/src/ ${GLPK_ROOT}/src/.libs/
        PATH_SUFFIXES "lib" "lib32" "lib64" "libglpk"
        NO_DEFAULT_PATH)

# look for library files, including default paths
find_library(GLPK_LIBRARY
        NAMES "glpk"
        PATH_SUFFIXES "lib" "lib32" "lib64" "libglpk")

# check version specific macros
include(CheckCSourceCompiles)
include(CMakePushCheckState)
cmake_push_check_state()

# we need if clauses here because variable is set variable-NOTFOUND
#
if (GLPK_INCLUDE_DIR)
    # message("Find the glpk include directory.")
    set(CMAKE_REQUIRED_INCLUDES ${CMAKE_REQUIRED_INCLUDES} ${GLPK_INCLUDE_DIR})
endif (GLPK_INCLUDE_DIR)
if (GLPK_LIBRARY)
    # message("Find the glpk library directory.")
    set(CMAKE_REQUIRED_LIBRARIES ${CMAKE_REQUIRED_LIBRARIES} ${GLPK_LIBRARY})
endif (GLPK_LIBRARY)

# handle package arguments
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(
        "GLPK"
        DEFAULT_MSG
        GLPK_INCLUDE_DIR
        GLPK_LIBRARY)

mark_as_advanced(GLPK_INCLUDE_DIR GLPK_LIBRARY)

# if both headers and library are found, store results
if (GLPK_FOUND)
    set(GLPK_INCLUDE_DIRS ${GLPK_INCLUDE_DIR})
    set(GLPK_LIBRARIES ${GLPK_LIBRARY})
    # log result
    file(APPEND ${CMAKE_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/CMakeOutput.log
            "Determining location of GLPK succeeded:\n"
            "Include directory: ${GLPK_INCLUDE_DIRS}\n"
            "Library directory: ${GLPK_LIBRARIES}\n\n")
else (GLPK_FOUND)
    # log errornous result
    file(APPEND ${CMAKE_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/CMakeError.log
            "Determining location of GLPK failed:\n"
            "Include directory: ${GLPK_INCLUDE_DIRS}\n"
            "Library directory: ${GLPK_LIBRARIES}\n\n")
endif (GLPK_FOUND)

set(SOURCE_FILES
        src/path/path.hpp
        src/path/path_bernoulli.hpp
        src/bandit/distributions.hpp
        src/bandit/roundwiselog.hpp
        src/bandit/simulator.hpp
        src/bandit/bandit_util.hpp
        src/policy/policy.hpp
        src/policy/policy_klucb.hpp
        src/policy/policy_exp3m.hpp
        src/policy/policy_random.hpp
        src/policy/policy_mpts.hpp
        src/lpsolver/matrix.h
        src/lpsolver/matrix.cpp
        src/lpsolver/lpsolver.h
        src/lpsolver/lpsolver.cpp
        src/cmdline.h
        src/bandit/init_util.hpp
        src/main.cpp src/bandit/macro_util.h
        src/path/path_normal.hpp
        src/network/client.hpp
        src/policy/policy_conmpts_latency.hpp
        src/policy/policy_conmpts_bandwidth.hpp
        src/policy/policy_conmpts_loss.hpp src/path/path_fixvalue.hpp)
add_executable(multi-path-selection ${SOURCE_FILES})
if( CMPTS_KERNEL )
    include_directories( ${CMPTS_KERNEL} )
    add_definitions( -DCMPTS_KERNEL )
    target_sources( multi-path-selection PRIVATE src/bandit/kernel_util.hpp )
endif()
target_link_libraries(multi-path-selection ${GLPK_LIBRARIES})
