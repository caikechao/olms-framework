//
// Created by kccai.
//

#pragma once

#include <vector>
#include <string>

class Matrix {
public:
    typedef std::vector<double> Row;

    Matrix(int size);

    Matrix(int size1, int size2);

    Matrix(const Matrix& s);

    ~Matrix();

    Matrix& operator=(const Matrix& s);

    void transpose();

    double trace();

    void add(const Matrix& s);

    void subtract(const Matrix& s);

    void multiply(const Matrix& s);

    Row& operator[](int pos);

    int numRows() const;

    void printMat(const std::string& name) const;

private:
    std::vector<Row> m_rows;
};

//
Matrix operator+(const Matrix& s1, const Matrix& s2);

Matrix operator-(const Matrix& s1, const Matrix& s2);

Matrix operator*(const Matrix& s1, const Matrix& s2);


