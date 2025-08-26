/*
 * Matrix Implementation
 *
 * Created by Paul Gabel
 * Date: 7/27/2025
 *
 * */

#ifndef MATRIX_HPP
#define MATRIX_HPP

#ifdef __cplusplus

#include <vector>
#include <cassert>
#include <cmath>

template<typename T>

/*
 * represents a 2d matrix as a vector, and allows for common matrix operations
 *
 * */
class Matrix {
public:
	// hold matrix dimensions
	uint8_t rows, cols;

	// data "flat packed" into vector
	std::vector<T> data;

	/*
	 * copy constructor
	 *
	 * */
	Matrix (const Matrix<T> &mat) : rows(mat.rows), cols(mat.cols), data(mat.data) {}

	/*
	 * constructor for uniform array
	 *
	 * */
	Matrix (uint8_t r, uint8_t c) : rows(r), cols(c), data(r*c) {}

	/*
	 * array constructor
	 *
	 * arr must be length r * c
	 *
	 * */
	Matrix (uint8_t r, uint8_t c, const T* arr) : rows(r), cols(c), data(arr, arr + r * c) {}

	/*
	 * array constructor, allow {} in function call
	 *
	 * */
	Matrix (uint8_t r, uint8_t c, std::initializer_list<T> arr) : rows(r), cols(c), data(arr) {}

	/*
	 * overload to allow function like use
	 *
	 * PARAMS i,j: positions in matrix, rows, cols
	 * counting starts at 1
	 *
	 * returns reference to data point allowing
	 *
	 * * first function allows write, second allows read, used const to be more efficient
	 *
	 * */
	T& operator()(uint8_t i, uint8_t j) {

		i--;
		j--;

		return data[i * cols + j];
	}

	const T& operator()(uint8_t i, uint8_t j) const {

		i--;
		j--;

		return data[i * cols + j];
	}

	// check matrix equality
	bool operator==(const Matrix<T>& other) const {

	    const T epsilon = {(T)0.001};

        if (rows != other.rows || cols != other.cols) return false;
        for (int i = 1; i <= rows; i++) {
            for (int j = 1; j <= cols; j++) {

                // use epsilon error bound to account for float
                if ((*this)(i, j) - other(i, j) > epsilon) return false;
            }
        }
        return true;
    }

	/*
	 * multiply this matrix by the one in parenthesis and return, this matrix remains unmodified
	 *
	 * PARAM other: matrix b in operation, unmodified
	 *
	 * returns the result matrix
	 * */
	Matrix<T> operator* (const Matrix<T>& other) const;

	/*
	 * multiply this matrix by scalar and return, this matrix remains unmodified
	 *
	 * PARAM other: matrix b in operation, unmodified
	 *
	 * returns the result matrix
	 * */
	Matrix<T> operator* (const T& scalar) const;

	/*
	 * add this matrix to the one in parenthesis, this matrix remains unmodified
	 *
	 * PARAM other: matrix b in operation, unmodified
	 *
	 * returns the result matrix
	 * */
	Matrix<T> operator+ (const Matrix<T>& other) const;

	/*
	 * subtract other from this matrix and return result, this matrix remains unmodified
	 *
	 * PARAM other: matrix b in operation, unmodified
	 *
	 * returns the result matrix
	 * */
	Matrix<T> operator- (const Matrix<T>& other) const;

	/*
	 * calculate transpose of this, self matrix remains unmodified
	 *
	 * returns the result matrix
	 * */
	Matrix<T> transpose () const;

	/*
	 * calculate inverse of this, self matrix remains unmodified
	 *
	 * returns the result matrix
	 * */
	Matrix<T> inverse () const;


private:
	/*
	 * swaps rows 1 and 2
	 *
	 * modifies this
	 *
	 * PARAM mat: matrix being operated on
	 * PARAM row1: row 1
	 * PARAM row2: row 2
	 *
	 * */
	void swap_rows (uint8_t row1, uint8_t row2);

	/*
	 * subtracts row2 from row1
	 *
	 * modifies this
	 *
	 * PARAM row1: modified row
	 * PARAM row2: operator row, unmodified
	 * PARAM multiplier: scales operator
	 *
	 * row1 = row2 * multiplier
	 *
	 * */
	void subtract_row (uint8_t row1, uint8_t row2, T multiplier = T{1});

};

#include "matrix.inl"


// allows for scalar to be on left, wraps matrix function above
template<typename T>
Matrix<T> operator* (T scalar, const Matrix<T>& mat){
	return mat * scalar;
}

#endif
#endif
