#include "eigen33.h"
#include <cmath>
#include <algorithm>
#include <numeric>
#include <numbers>

void computeEigenVectors3x3(const double A[3][3], double eigenvalues[3], double eigenvectors[3][3]) {
    double M[3][3] = {
        {A[0][0], A[0][1], A[0][2]},
        {A[1][0], A[1][1], A[1][2]},
        {A[2][0], A[2][1], A[2][2]}
    };

    eigenvectors[0][0] = 1; eigenvectors[0][1] = 0; eigenvectors[0][2] = 0;
    eigenvectors[1][0] = 0; eigenvectors[1][1] = 1; eigenvectors[1][2] = 0;
    eigenvectors[2][0] = 0; eigenvectors[2][1] = 0; eigenvectors[2][2] = 1;

    for (int iter = 0; iter < 50; ++iter) {
        int p = 0, q = 1;
        double max_val = std::abs(M[0][1]);
        
        if (std::abs(M[0][2]) > max_val) { max_val = std::abs(M[0][2]); p = 0; q = 2; }
        if (std::abs(M[1][2]) > max_val) { max_val = std::abs(M[1][2]); p = 1; q = 2; }

        if (max_val < 1e-10) break;

        double theta;
        if (M[p][p] == M[q][q]) {
            theta = std::numbers::pi / 4.0;
        } else {
            theta = 0.5 * std::atan2(2.0 * M[p][q], M[p][p] - M[q][q]);
        }

        double c = std::cos(theta);
        double s = std::sin(theta);

        double M_new[3][3];
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                M_new[i][j] = M[i][j];
            }
        }

        M_new[p][p] = c * c * M[p][p] - 2.0 * s * c * M[p][q] + s * s * M[q][q];
        M_new[q][q] = s * s * M[p][p] + 2.0 * s * c * M[p][q] + c * c * M[q][q];
        M_new[p][q] = 0.0;
        M_new[q][p] = 0.0;

        for (int i = 0; i < 3; ++i) {
            if (i != p && i != q) {
                M_new[i][p] = c * M[i][p] - s * M[i][q];
                M_new[p][i] = M_new[i][p];
                M_new[i][q] = s * M[i][p] + c * M[i][q];
                M_new[q][i] = M_new[i][q];
            }
        }

        for (int i = 0; i < 3; ++i) {
            double vip = eigenvectors[i][p];
            double viq = eigenvectors[i][q];
            eigenvectors[i][p] = c * vip - s * viq;
            eigenvectors[i][q] = s * vip + c * viq;
        }

        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                M[i][j] = M_new[i][j];
            }
        }
    }

    eigenvalues[0] = M[0][0];
    eigenvalues[1] = M[1][1];
    eigenvalues[2] = M[2][2];

    if (eigenvalues[0] > eigenvalues[1]) {
        std::swap(eigenvalues[0], eigenvalues[1]);
        for(int i=0; i<3; ++i) std::swap(eigenvectors[i][0], eigenvectors[i][1]);
    }
    if (eigenvalues[1] > eigenvalues[2]) {
        std::swap(eigenvalues[1], eigenvalues[2]);
        for(int i=0; i<3; ++i) std::swap(eigenvectors[i][1], eigenvectors[i][2]);
    }
    if (eigenvalues[0] > eigenvalues[1]) {
        std::swap(eigenvalues[0], eigenvalues[1]);
        for(int i=0; i<3; ++i) std::swap(eigenvectors[i][0], eigenvectors[i][1]);
    }
}