#include "matrix4x4.h"
#include <string.h>

// 单位矩阵
Matrix4x4 matrix4x4_identity() {
    Matrix4x4 mat;
    memset(&mat, 0, sizeof(mat));
    mat.data[0][0] = mat.data[1][1] = mat.data[2][2] = mat.data[3][3] = 1.0f;
    return mat;
}

// 4x4 × 4x4 乘法
Matrix4x4 matrix4x4_multiply(const Matrix4x4 *a, const Matrix4x4 *b) {
    Matrix4x4 result;
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            result.data[i][j] = 0.0f;
            for (int k = 0; k < 4; k++) {
                result.data[i][j] += a->data[i][k] * b->data[k][j];
            }
        }
    }
    return result;
}

// 4x4 × 4x1 乘法
Vector4 matrix4x4_multiply_vector(const Matrix4x4 *mat, const Vector4 *vec) {
    Vector4 result;
    for (int i = 0; i < 4; i++) {
        result.data[i] = 0.0f;
        for (int j = 0; j < 4; j++) {
            result.data[i] += mat->data[i][j] * vec->data[j];
        }
    }
    return result;
}

// 矩阵加法
Matrix4x4 matrix4x4_add(const Matrix4x4 *a, const Matrix4x4 *b) {
    Matrix4x4 result;
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            result.data[i][j] = a->data[i][j] + b->data[i][j];
        }
    }
    return result;
}

// 矩阵转置
Matrix4x4 matrix4x4_transpose(const Matrix4x4 *mat) {
    Matrix4x4 result;
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            result.data[i][j] = mat->data[j][i];
        }
    }
    return result;
}

// 矩阵求逆（仅适用于刚体变换矩阵）
Matrix4x4 matrix4x4_inverse(const Matrix4x4 *mat) {
    Matrix4x4 inv;
    // 旋转部分（3x3）的逆 = 转置
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            inv.data[i][j] = mat->data[j][i];
        }
    }
    // 平移部分的逆
    float tx = mat->data[0][3], ty = mat->data[1][3], tz = mat->data[2][3];
    inv.data[0][3] = -(inv.data[0][0]*tx + inv.data[0][1]*ty + inv.data[0][2]*tz);
    inv.data[1][3] = -(inv.data[1][0]*tx + inv.data[1][1]*ty + inv.data[1][2]*tz);
    inv.data[2][3] = -(inv.data[2][0]*tx + inv.data[2][1]*ty + inv.data[2][2]*tz);
    // 齐次部分
    inv.data[3][0] = inv.data[3][1] = inv.data[3][2] = 0.0f;
    inv.data[3][3] = 1.0f;
    return inv;
}


// 3x3单位矩阵
Matrix3x3 matrix3x3_identity() {
    Matrix3x3 mat;
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            mat.data[i][j] = (i == j) ? 1.0f : 0.0f;
        }
    }
    return mat;
}

// 3x3 × 3x3 乘法
Matrix3x3 matrix3x3_multiply(const Matrix3x3 *a, const Matrix3x3 *b) {
    Matrix3x3 result;
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            result.data[i][j] = 0.0f;
            for (int k = 0; k < 3; k++) {
                result.data[i][j] += a->data[i][k] * b->data[k][j];
            }
        }
    }
    return result;
}

// 3x3 × 3x1 乘法
Vector3 matrix3x3_multiply_vector(const Matrix3x3 *mat, const Vector3 *vec) {
    Vector3 result;
    for (int i = 0; i < 3; i++) {
        result.data[i] = 0.0f;
        for (int j = 0; j < 3; j++) {
            result.data[i] += mat->data[i][j] * vec->data[j];
        }
    }
    return result;
}

// 3x3矩阵加法
Matrix3x3 matrix3x3_add(const Matrix3x3 *a, const Matrix3x3 *b) {
    Matrix3x3 result;
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            result.data[i][j] = a->data[i][j] + b->data[i][j];
        }
    }
    return result;
}

// 3x3矩阵减法
Matrix3x3 matrix3x3_subtract(const Matrix3x3 *a, const Matrix3x3 *b) {
    Matrix3x3 result;
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            result.data[i][j] = a->data[i][j] - b->data[i][j];
        }
    }
    return result;
}

// 3x3矩阵转置
Matrix3x3 matrix3x3_transpose(const Matrix3x3 *mat) {
    Matrix3x3 result;
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            result.data[i][j] = mat->data[j][i];
        }
    }
    return result;
}

// 3x3矩阵行列式
float matrix3x3_determinant(const Matrix3x3 *mat) {
    return mat->data[0][0] * (mat->data[1][1]*mat->data[2][2] - mat->data[1][2]*mat->data[2][1])
         - mat->data[0][1] * (mat->data[1][0]*mat->data[2][2] - mat->data[1][2]*mat->data[2][0])
         + mat->data[0][2] * (mat->data[1][0]*mat->data[2][1] - mat->data[1][1]*mat->data[2][0]);
}

// 3x3矩阵求逆
Matrix3x3 matrix3x3_inverse(const Matrix3x3 *mat) {
    Matrix3x3 inv;
    float det = matrix3x3_determinant(mat);

    if (fabsf(det) < 1e-6f) {
        // 奇异矩阵，返回单位矩阵
        return matrix3x3_identity();
    }

    float inv_det = 1.0f / det;

    inv.data[0][0] = (mat->data[1][1]*mat->data[2][2] - mat->data[1][2]*mat->data[2][1]) * inv_det;
    inv.data[0][1] = (mat->data[0][2]*mat->data[2][1] - mat->data[0][1]*mat->data[2][2]) * inv_det;
    inv.data[0][2] = (mat->data[0][1]*mat->data[1][2] - mat->data[0][2]*mat->data[1][1]) * inv_det;

    inv.data[1][0] = (mat->data[1][2]*mat->data[2][0] - mat->data[1][0]*mat->data[2][2]) * inv_det;
    inv.data[1][1] = (mat->data[0][0]*mat->data[2][2] - mat->data[0][2]*mat->data[2][0]) * inv_det;
    inv.data[1][2] = (mat->data[0][2]*mat->data[1][0] - mat->data[0][0]*mat->data[1][2]) * inv_det;

    inv.data[2][0] = (mat->data[1][0]*mat->data[2][1] - mat->data[1][1]*mat->data[2][0]) * inv_det;
    inv.data[2][1] = (mat->data[0][1]*mat->data[2][0] - mat->data[0][0]*mat->data[2][1]) * inv_det;
    inv.data[2][2] = (mat->data[0][0]*mat->data[1][1] - mat->data[0][1]*mat->data[1][0]) * inv_det;

    return inv;
}


Matrix2x2 matrix2x2_identity(void) {
    Matrix2x2 mat;
    mat.data[0][0] = 1.0f;
    mat.data[0][1] = 0.0f;
    mat.data[1][0] = 0.0f;
    mat.data[1][1] = 1.0f;
    return mat;
}

Matrix2x2 matrix2x2_multiply(const Matrix2x2 *a, const Matrix2x2 *b) {
    Matrix2x2 result;
    result.data[0][0] = a->data[0][0] * b->data[0][0] + a->data[0][1] * b->data[1][0];
    result.data[0][1] = a->data[0][0] * b->data[0][1] + a->data[0][1] * b->data[1][1];
    result.data[1][0] = a->data[1][0] * b->data[0][0] + a->data[1][1] * b->data[1][0];
    result.data[1][1] = a->data[1][0] * b->data[0][1] + a->data[1][1] * b->data[1][1];
    return result;
}

Vector2 matrix2x2_multiply_vector(const Matrix2x2 *mat, const Vector2 *vec) {
    Vector2 result;
    result.data[0] = mat->data[0][0] * vec->data[0] + mat->data[0][1] * vec->data[1];
    result.data[1] = mat->data[1][0] * vec->data[0] + mat->data[1][1] * vec->data[1];
    return result;
}

Matrix2x2 matrix2x2_transpose(const Matrix2x2 *mat) {
    Matrix2x2 result;
    result.data[0][0] = mat->data[0][0];
    result.data[0][1] = mat->data[1][0];
    result.data[1][0] = mat->data[0][1];
    result.data[1][1] = mat->data[1][1];
    return result;
}

float matrix2x2_determinant(const Matrix2x2 *mat) {
    return mat->data[0][0] * mat->data[1][1] - mat->data[0][1] * mat->data[1][0];
}

Matrix2x2 matrix2x2_inverse(const Matrix2x2 *mat) {
    float det = matrix2x2_determinant(mat);
    if (fabsf(det) < 1e-6f) {
        return matrix2x2_identity();
    }
    float inv_det = 1.0f / det;
    Matrix2x2 inv;
    inv.data[0][0] =  mat->data[1][1] * inv_det;
    inv.data[0][1] = -mat->data[0][1] * inv_det;
    inv.data[1][0] = -mat->data[1][0] * inv_det;
    inv.data[1][1] =  mat->data[0][0] * inv_det;
    return inv;
}


