#pragma once

#include "Control/matrix.hpp"

template<typename T>
Matrix<T> Matrix<T>::operator* (const Matrix<T>& other) const{

	assert (cols == other.rows);

	Matrix<T> result (rows, other.cols);

	// loop over result matrix
	for (uint8_t i = 1; i <= rows; i++) {
		for (uint8_t j = 1; j <= other.cols; j++) {

			// for each individual multiplication operation add to result element
			for (uint8_t k = 1; k <= cols; k++) {
				result (i,j) += (*this)(i, k) * other(k,j);
			}
		}
	}

	return result;
}

template<typename T>
Matrix<T> Matrix<T>::operator* (const T& scalar) const{

	Matrix<T> result (rows, cols);

	// loop over result matrix
	for (uint8_t i = 1; i <= rows; i++) {
		for (uint8_t j = 1; j <= cols; j++) {
			result (i,j) = (*this)(i, j) * scalar;
		}
	}

	return result;
}

template<typename T>
Matrix<T> Matrix<T>::operator+ (const Matrix<T>& other) const{

	assert (cols == other.cols);
	assert (rows == other.rows);

	Matrix<T> result (rows, cols);

	// loop over result matrix
	for (uint8_t i = 1; i <= rows; i++) {
		for (uint8_t j = 1; j <= cols; j++) {
			result (i,j) = (*this)(i, j) + other(i, j);
		}
	}

	return result;
}


template<typename T>
Matrix<T> Matrix<T>::operator- (const Matrix<T>& other) const{

	assert (cols == other.cols);
	assert (rows == other.rows);

	Matrix<T> result (rows, cols);

	// loop over result matrix
	for (uint8_t i = 1; i <= rows; i++) {
		for (uint8_t j = 1; j <= cols; j++) {
			result (i,j) = (*this)(i, j) - other(i, j);
		}
	}

	return result;
}

template<typename T>
Matrix<T> Matrix<T>::transpose () const{

	Matrix<T> result (cols, rows);

	// loop over result matrix
	for (uint8_t i = 1; i <= rows; i++) {
		for (uint8_t j = 1; j <= cols; j++) {
			result (j, i) = (*this)(i, j);
		}
	}

	return result;
}

template<typename T>
Matrix<T> Matrix<T>::inverse () const{

	assert (rows == cols);

    // Create augmented matrix: rows x 2rows
    Matrix<T> aug(rows, 2 * rows);

    // Initialize aug with [A | I]
    for (uint8_t i = 1; i <= rows; i++) {
        for (uint8_t j = 1; j <= rows; j++) {

            aug(i, j) = (*this)(i, j);
            aug(i, j + rows) = (i == j) ? T{1} : T{0};
        }
    }

    // Forward elimination
    for (uint8_t i = 1; i <= rows; i++) {

        // Find pivot in column i, starting from row i
    	uint8_t pivot = i;

        for (uint8_t r = i + 1; r <= rows; r++) {
            if (abs(aug(r, i)) > std::abs(aug(pivot, i))) {
                pivot = r;
            }
        }

        // cannot invert matrix, raise an error
        assert (aug(pivot, i) != T{0});

        // Swap current row with pivot row if needed
        if (pivot != i) {
            aug.swap_rows(i, pivot);
        }

        // Normalize pivot row
        T pivotVal = aug(i, i);

        for (uint8_t col = 1; col <= 2 * rows; col++) {
            aug(i, col) = aug(i, col) / pivotVal;
        }

        // Eliminate rows below
        for (uint8_t r = i + 1; r <= rows; r++) {
            T factor = aug(r, i);
            if (factor != T{0}) {

                // subtract_row modifies row r by subtracting row i * factor
                // But subtract_row subtracts row2 * multiplier from row1,
                // so call subtract_row(row1=r, row2=i, multiplier=factor)
                aug.subtract_row(r, i, factor);
            }
        }
    }

    // Backward elimination
    for (uint8_t i = rows; i >= 1; i--) {
        for (uint8_t r = i - 1; r >= 1; r--) {

            T factor = aug(r, i);
            if (factor != T{0}) {

                aug.subtract_row(r, i, factor);
            }
        }
    }

    // Extract right half as inverse
    Matrix<T> inv(rows, cols);

    for (uint8_t i = 1; i <= rows; i++) {
        for (uint8_t j = 1; j <= cols; j++) {

            inv(i, j) = aug(i, j + cols);
        }
    }

    return inv;

}

template<typename T>
void Matrix<T>::swap_rows (uint8_t row1, uint8_t row2) {

	for (uint8_t i = 1; i <= (*this).cols; i++) {

		std::swap((*this)(row1, i), (*this)(row2, i));
	}


}

template<typename T>
void Matrix<T>::subtract_row (uint8_t row1, uint8_t row2, T multiplier) {

	// for each column in row1, subtract row2 * multiplier
	for (uint8_t i = 1; i <= (*this).cols; i++) {
		(*this)(row1, i) = (*this)(row1, i) - multiplier *  (*this)(row2, i);
	}
}
