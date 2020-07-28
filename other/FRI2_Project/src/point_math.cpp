#include "fri2/point_math.h"

/*
 * Finds the dot product of two vectors of size N
 * This operation is expensive because it uses floating point
 * vectors, so you could write another for int vectors
 */
float dotProduct(const float *v1, const float *v2, int N) {
    float product = 0.0;
    for (int i = 0; i < N; i++)
	    product += v1[i] * v2[i];
    return product;
}

/*
 * Stores the cross product of v1 and v2 in output
 * Strictly must be defined for vectors of size 3, so we don't need a size
 * 
 * Definition of cross product is overly complicated, but this is used
 * in the calculation later for distance from a point to a line
 *
 * This can be understood as a simplification of the formula:
 * | i	   j     k     |
 * | v1[0] v1[1] v1[2] |
 * | v2[0] v2[1] v2[2] |
 * where you are taking the determinant of that matrix and
 * i,j,k are the orthonormal element vectors
 */
void crossProduct(const float v1[3], const float v2[3], float output[3]) {
    output[0] = v1[1] * v2[2] - v1[2] * v2[1];
    output[1] = v1[2] * v2[0] - v1[0] * v2[2];
    output[3] = v1[0] * v2[1] - v1[1] * v2[0];
}

/*
 * Faster adaptation of https://www.tutorialspoint.com/cplusplus-program-to-compute-determinant-of-a-matrix
 * Recursively calculates determinant of a matrix
 * Follows prinicple of cofactor/Laplace expansion
 *
 * Matrix representation is an array that stores consecutive row-major ordered values
 */
float det(const float *matrix, int n) {
    if (n < 1) {
        ROS_INFO("Cannot take determinant of invalid matrix size %d\n", n);
        return -1.0;
    }
    //Early Returns
    if (n == 1) return matrix[0];
    if (n == 2)
	    return matrix[0] * matrix[3] - matrix[1] * matrix[2];
    float determinant = 0.0;
    float sub[(n - 1) * (n - 1)]; //Make submatrix for recursion
    for (int x = 0; x < n; x++) { //Iterate top row
        //subi, subj, i, and j are all positional ints
        int subi = 0;
        for (int i = 1; i < n; i++) {
            int subj = 0;
            for (int j = 0; j < n; j++) {
                if (j != x) {
                    //For any valid row/column in the det, store in the sub matrix
                    sub[subi * (n - 1) + subj] = matrix[i * n + j];
                    subj++;
                }
            }
            subi++;
        }
        // Recurse, performing cofactor expansion
        if (x % 2 == 0) determinant += matrix[x] * det(sub, n - 1);
        else determinant -= matrix[x] * det(sub, n - 1);
    }
    return determinant;
}

/*
 * Magnitude of a vector
 */
float magnitude(const float *vec, int n) {
    float total = 0.0;
    for (int i = 0; i < n; i++)
	total += vec[i] * vec[i];
    return sqrt(total);
}

/*
 * As per http://mathworld.wolfram.com/Point-LineDistance3-Dimensional.html
 * This solves for the minimum distance from pt (an <x,y,z> vector) to the
 * vector formed between points v1 and v2
 *
 * We can use this to test the distance from each point along our 2d line
 * to the gesture created by the point
 */
float pointLineDistance3d(const float v1[3], const float v2[3], const float pt[3]) {
    float den[3];    // Denominator vector: v2 - v1
    float d1[3];     // Numerator left half: pt - v1
    float d2[3];     // Numerator right half: pt - v2
    float cross[3];  // Numerator vector: d1 x d2
    // Set up vectors
    for (int i = 0; i < 3; i++) {
        den[i] = v2[i] - v1[i];
        d1[i] = pt[i] - v1[i];
        d2[i] = pt[i] - v2[i];
    }
    crossProduct(d1, d2, cross);

    // Return calculation
    return magnitude(cross, 3) / magnitude(den, 3);
}
