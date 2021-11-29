
#pragma once
#include "macro_util.h"
#include <vector>
#include <array>
#include <string>
#include <set>
#include <map>
#include <queue>
#include <algorithm>
#include <memory>
#include <fstream>
#include <sstream>
#include <iostream>
#include <iomanip>
#include <random>
#include <cmath>
#include <ctime>
#include <cfloat>

namespace bandit {

typedef unsigned int uint;

typedef std::vector<std::vector<double> > vec2Double;

typedef std::vector<std::vector<std::vector<double> > > vec3Double;
typedef std::vector<std::vector<std::vector<uint> > > vec3Uint;

const double pi = 3.14159265;

const double e = 2.718281828;

//RNG engine
std::mt19937 randomEngine(std::time(0));

double bernoulliTrial(double rand, double mu)
{
    if (rand<mu) {
        return 1.0;
    }
    else {
        return 0.0;
    }
};

struct Measurement {
    int path_id;
    double r;
    double b;
    double l;

    Measurement(int path_id = -1, double r = 0.0, double b = 0.0, double l = 0.0)
            :path_id(path_id), r(r), b(b), l(l)
    {
    }

};

struct Metric {

    double r;
    double b;
    double l;

    // default + parameterized constructor
    Metric(double r = 0.0, double b = 0.0, double l = 0.0)
            :r(r), b(b), l(l)
    {
    }

    Metric& operator=(const Metric& pa)
    {
        r = pa.r;
        b = pa.b;
        l = pa.l;
        return *this;
    }

    Metric& operator+=(const Metric& pa)
    {
        r += pa.r;
        b += pa.b;
        l += pa.l;
        return *this;
    }

    Metric operator+(const Metric& pa) const
    {
        return Metric(pa.r+r, pa.b+b, pa.l+l);
    }

    Metric averageOver(const uint n) const
    {
        return Metric(r/n, b/n, l/n);
    }

};

typedef std::vector<std::vector<std::vector<Metric> > > vec3Metric;

void printMsg(const std::string& msg)
{
#if DEBUG_mode
    std::cout << msg << std::endl;
#endif
}

template<class T>
void printVec(const std::string& name, const std::vector<T>& vec)
{
#if DEBUG_mode
    std::cout << name+": ";
    for (const auto& i:vec) {
        std::cout << i << " ";
    }
    std::cout << std::endl;
#endif
}

template<class T>
void printVecR(const std::string& name, const std::vector<T>& vec)
{
    std::cout << name+": ";
    for (const auto& i:vec) {
        std::cout << i << " ";
    }
    std::cout << std::endl;
}

template<class T>
void printVar(const std::string& name, const T& var)
{
#if DEBUG_mode
    std::cout << name+": ";
    std::cout << var << " ";
    std::cout << std::endl;
#endif
}

template<class T>
uint vectorMaxIndex(const std::vector<T>& elems)
{
    uint m = 0;
    T mv = elems[0];
    for (uint i = 0; i<elems.size(); ++i) {
        if (elems[i]>mv) {
            mv = elems[i];
            m = i;
        }
    }
    return m;
}

template<class T>
std::vector<uint> vectorMaxIndices(const std::vector<T>& elems, uint l)
{
    std::vector<std::pair<T, uint>> elemPairs;
    for (uint i = 0; i<elems.size(); ++i) {
        elemPairs.push_back(std::make_pair(-elems[i], i));
    }
    std::stable_sort(elemPairs.begin(), elemPairs.end());
    std::vector<uint> is;
    uint i = 0;
    for (auto it = elemPairs.begin(); (it!=elemPairs.end()) && (i<l); ++it, ++i) {
        is.push_back(it->second);
    }
    return is;
}

template<class T>
std::vector<uint> vectorMinIndices(const std::vector<T>& elems, uint l)
{
    std::vector<std::pair<T, uint>> elemPairs;
    for (uint i = 0; i<elems.size(); ++i) {
        elemPairs.push_back(std::make_pair(elems[i], i));// reverse the sign
    }
    std::stable_sort(elemPairs.begin(), elemPairs.end());
    std::vector<uint> is;
    uint i = 0;
    for (auto it = elemPairs.begin(); (it!=elemPairs.end()) && (i<l); ++it, ++i) {
        is.push_back(it->second);
    }
    return is;
}

template<class T>
T vectorMax(const std::vector<T>& elems)
{
    T mv = elems[0];
    for (unsigned int i = 0; i<elems.size(); ++i) {
        if (elems[i]>mv) {
            mv = elems[i];
        }
    }
    return mv;
}

template<class T>
T vectorSum(const std::vector<T>& elems)
{
    T s = 0;
    for (uint i = 0; i<elems.size(); ++i) {
        s += elems[i];
    }
    return s;
}

template<class T>
std::vector<uint> vectorLTHIndices(const std::vector<T>& elems, double h)
{
    std::vector<uint> indices;
    for (uint i = 0; i<elems.size(); ++i) {
        if (elems[i]<=h) {
            indices.push_back(i);
        }
    }
    return indices;
}

template<class T>
std::vector<T> newVectorWithIndices(const std::vector<T>& elems, const std::vector<uint>& indices)
{
    std::vector<T> newVec;
    for (uint i = 0; i<indices.size(); ++i) {
        uint k = indices[i];
        newVec.push_back(elems[k]);
    }
    return newVec;
}

template<class T>
T vectorDot(const std::vector<T>& vec1, const std::vector<T>& vec2)
{
    T s = 0;
    if (vec1.size()!=vec2.size()) {
        std::cerr << "Error in vectorDot: vector sizes are not equal. Vec1.size=" << vec1.size()
                  << " Vec2.size=" << vec2.size()
                  << std::endl;
        abort();
    }
    for (uint i = 0; i<vec1.size(); ++i) {
        s += vec1[i]*vec2[i];
    }
    return s;
}

//int -> string
std::string itos(int number)
{
    std::stringstream ss;
    ss << number;
    return ss.str();
}

//double -> string
std::string dtos(double number)
{
    std::stringstream ss;
    ss << number;
    return ss.str();
}

//randomly sort 1,...,K
std::vector<uint> randomIndices(uint K)
{
    std::vector<std::pair<double, uint>> temp;
    for (uint i = 0; i<K; ++i) {
        temp.push_back(std::make_pair(std::uniform_real_distribution<double>(0.0, 1.0)(randomEngine), i));
    }
    std::sort(temp.begin(), temp.end());
    std::vector<uint> rvec;
    for (uint i = 0; i<K; ++i) {
        rvec.push_back(temp[i].second);
    }
    return rvec;
}

template<class T>
double kl(T p, T q)
{
    return p*(log(p)-log(q))+(1-p)*(log(1-p)-log(1-q));
}

template<class T>
double max(T p, T q)
{
    if (p>q) {
        return p;
    }
    else {
        return q;
    }
}

//split by char
std::vector<std::string> split(const std::string& str, char delim)
{
    std::vector<std::string> res;
    int current = 0, found;
    while ((found = str.find_first_of(delim, current))!=(int) (std::string::npos)) {
        res.push_back(std::string(str, current, found-current));
        current = found+1;
    }
    res.push_back(std::string(str, current, str.size()-current));
    return res;
}

//split by string
std::vector<std::string> split2(const std::string& str, const std::string& delim)
{
    std::vector<std::string> res;
    int current = 0, found, delimlen = (int) delim.size();
    while ((found = str.find(delim, current))!=(int) (std::string::npos)) {
        res.push_back(std::string(str, current, found-current));
        current = found+delimlen;
    }
    res.push_back(std::string(str, current, str.size()-current));
    return res;
}

std::vector<std::string> readlines(const std::string filename, bool skipBlankLine = true)
{
    std::ifstream ifs(filename);
    if (!ifs.is_open()) {
        std::cerr << "ERROR: File " << filename << " not found." << std::endl;
        exit(0);
    }
    std::vector<std::string> lines;
    std::string str;
    while (std::getline(ifs, str)) {
        if (skipBlankLine && (str.size()==0)) continue;
        lines.push_back(str);
    }
    ifs.close();
    return lines;
}

template<class key, class T>
key randomKeyFromMap(const std::map<key, T>& m)
{
    uint l = m.size();
    uint i = std::uniform_int_distribution<int>(0, l-1)(randomEngine);
    typename std::map<key, T>::const_iterator it(m.begin());
    std::advance(it, i);
    return it->first;
}

template<class T>
T randomElementFromSet(const std::set<T>& s)
{
    uint l = s.size();
    uint i = std::uniform_int_distribution<int>(0, l-1)(randomEngine);
    typename std::set<T>::const_iterator it(s.begin());
    std::advance(it, i);
    return *it;
}

template<class T>
std::vector<T> randomElementsFromVector(const std::vector<T>& v, uint num)
{
    const uint num_check = num;
    if (num>v.size()) {
        std::cout << "Error: specified element num is larger than vector size!" << std::endl;
    }
    std::set<uint> S;
    for (uint i = 0; i<v.size(); ++i) {
        S.insert(i);
    }
    std::vector<double> elems;
    while (num>0) {
        uint i = randomElementFromSet(S);
        S.erase(i);
        elems.push_back(v[i]);
        num--;
    }
    if (elems.size()!=num_check) {
        std::cout << "Error in randomElementsFromVector" << std::endl;
        exit(0);
    }
    return elems;
}

std::string doubleSetToString(std::set<double> s)
{
    std::string astr = "";
    for (std::set<double>::iterator it = s.begin(); it!=s.end(); it++) {
        if (astr.length()==0) {
            astr += itos(*it);
        }
        else {
            astr += std::string(" ")+dtos(*it);
        }
    }
    return astr;
}

std::vector<uint> dependentRounding(uint l, std::vector<double> ps)
{
    uint K = (uint) ps.size();
    printMsg("DependentRounding start");
    if (std::abs(vectorSum(ps)-l)>0.001) {

        std::cerr << "Error: probability sum is not equal to " << l << " (" << vectorSum(ps) << ")"
                  << std::endl;
        abort();
    }
    std::vector<int> selected(K, -1); //-1-> not determined 0->not selected 1->selected
    for (uint i = 0; i<K; ++i) {
        if (ps[i]>0.999) {
            selected[i] = 1;
        }
        else if (ps[i]<0.001) {
            selected[i] = 0;
        }
    }
    std::vector<uint> randIndices = randomIndices(K);
    while (true) {
        std::pair<uint, uint> ij;
        bool iSelected = false;
        bool iSelected2 = false;
        uint i;
        for (i = 0; i<K; ++i) {
            //std::cout << "itemp=" << i << std::endl;
            uint ind = randIndices[i];
            if (!iSelected) {
                if (selected[ind]==-1) {
                    ij.first = ind;
                    iSelected = true;
                }
            }
            else {
                if (selected[ind]==-1) {
                    ij.second = ind;
                    iSelected2 = true;
                    break;
                }
            }
        }
        if ((i==K) && (iSelected==false)) {
            break;
        }
        if (i==K && (!iSelected2)) {
            if (ps[ij.first]>0.5) {
                selected[ij.first] = 1;
            }
            else {
                selected[ij.first] = 0;
            }
            break;
        }
        std::pair<double, double> pij;
        pij.first = ps[ij.first];
        pij.second = ps[ij.second];
        double alpha = std::min(1-pij.first, pij.second);
        double beta = std::min(pij.first, 1-pij.second);
        double prob = beta/(alpha+beta);
        double rand = std::uniform_real_distribution<double>(0.0, 1.0)(randomEngine);
        double pin, pjn;
        if (rand<prob) {
            pin = pij.first+alpha;
            pjn = pij.second-alpha;
        }
        else {
            pin = pij.first-beta;
            pjn = pij.second+beta;
        }
        if (pin>0.999) {
            selected[ij.first] = 1;
        }
        else if (pin<0.001) {
            selected[ij.first] = 0;
        }
        else {
            ps[ij.first] = pin;
        }
        if (pjn>0.999) {
            selected[ij.second] = 1;
        }
        else if (pjn<0.001) {
            selected[ij.second] = 0;
        }
        else {
            ps[ij.second] = pjn;
        }
    }
    std::vector<uint> selectedPaths;
    for (uint i = 0; i<K; ++i) {
        if (selected[i]==1) {
            selectedPaths.push_back(i);
        }
    }
    if (selectedPaths.size()!=l) {
        std::cerr << "Error: " << selectedPaths.size() << " selected (should be " << l << " )." << std::endl;
        abort();
    }
    printMsg("DependentRounding end!");
    return selectedPaths;
}

} //namespace
