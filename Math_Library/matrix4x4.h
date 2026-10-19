#ifndef MATRIX4X4_H
#define MATRIX4X4_H

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
// 4x4 矩阵
typedef struct {
    float data[4][4];  // [行][列]
} Matrix4x4;

// 4x1 向量（齐次坐标）
typedef struct {
    float data[4];     // [x, y, z, w]
} Vector4;
//临时加的3X3 矩阵
typedef struct {
    float data[3][3];
} Matrix3x3;
typedef struct {
    float data[2][2];
} Matrix2x2;
//2X2矩阵
//3X1向量
typedef struct {
    float data[3];
} Vector3;

typedef struct {
    float data[2];
} Vector2;
// 初始化为单位矩阵
Matrix4x4 matrix4x4_identity();

// 矩阵乘法：4x4 × 4x4
Matrix4x4 matrix4x4_multiply(const Matrix4x4 *a, const Matrix4x4 *b);

// 矩阵乘法：4x4 × 4x1
Vector4 matrix4x4_multiply_vector(const Matrix4x4 *mat, const Vector4 *vec);

// 矩阵加法
Matrix4x4 matrix4x4_add(const Matrix4x4 *a, const Matrix4x4 *b);

// 矩阵转置
Matrix4x4 matrix4x4_transpose(const Matrix4x4 *mat);

// 矩阵求逆（仅适用于刚体变换矩阵）
Matrix4x4 matrix4x4_inverse(const Matrix4x4 *mat);


Matrix3x3 matrix3x3_identity();
Matrix3x3 matrix3x3_multiply(const Matrix3x3 *a, const Matrix3x3 *b);
Vector3 matrix3x3_multiply_vector(const Matrix3x3 *mat, const Vector3 *vec);
Matrix3x3 matrix3x3_add(const Matrix3x3 *a, const Matrix3x3 *b);
Matrix3x3 matrix3x3_subtract(const Matrix3x3 *a, const Matrix3x3 *b);
Matrix3x3 matrix3x3_transpose(const Matrix3x3 *mat);
Matrix3x3 matrix3x3_inverse(const Matrix3x3 *mat);

Matrix2x2 matrix2x2_identity(void);
Matrix2x2 matrix2x2_multiply(const Matrix2x2 *a, const Matrix2x2 *b);
Vector2 matrix2x2_multiply_vector(const Matrix2x2 *mat, const Vector2 *vec);
Matrix2x2 matrix2x2_transpose(const Matrix2x2 *mat);
float matrix2x2_determinant(const Matrix2x2 *mat);
Matrix2x2 matrix2x2_inverse(const Matrix2x2 *mat);

#endif // MATRIX4X4_H
