/*! @file cTypes.h
 *  @brief Common types that are only valid in C++
 *
 *  This file contains types which are only used in C++ code.  This includes
 * Eigen types, template types, aliases, ...
 */

#ifndef PROJECT_CPPTYPES_H
#define PROJECT_CPPTYPES_H

#include <vector>
#include <Eigen/Dense>

// Rotation Matrix
template <typename T>
using RotMat = typename Eigen::Matrix<T, 3, 3>;

// 2x1 Vector
template <typename T>
using Vec2 = typename Eigen::Matrix<T, 2, 1>;

// 3x1 Vector
template <typename T>
using Vec3 = typename Eigen::Matrix<T, 3, 1>;

// 4x1 Vector
template <typename T>
using Vec4 = typename Eigen::Matrix<T, 4, 1>;

// 6x1 Vector
template <typename T>
using Vec6 = Eigen::Matrix<T, 6, 1>;

// 10x1 Vector
template <typename T>
using Vec10 = Eigen::Matrix<T, 10, 1>;

// 12x1 Vector
template <typename T>
using Vec12 = Eigen::Matrix<T, 12, 1>;

// 18x1 Vector
template <typename T>
using Vec18 = Eigen::Matrix<T, 18, 1>;

// 28x1 vector
template <typename T>
using Vec28 = Eigen::Matrix<T, 28, 1>;

// 78x1 vector
template <typename T>
using Vec78 = Eigen::Matrix<T, 78, 1>;

// 316x1 vector
template <typename T>
using Vec316 = Eigen::Matrix<T, 316, 1>;

// 316x1 vector
template <typename T>
using Vec316 = Eigen::Matrix<T, 316, 1>;

// 400x1 vector
template <typename T>
using Vec400 = Eigen::Matrix<T, 400, 1>;

// 500x1 vector
template <typename T>
using Vec500 = Eigen::Matrix<T, 500, 1>;

// Dynamically sized vector
template <typename T>
using DVec = typename Eigen::Matrix<T, Eigen::Dynamic, 1>;

#endif
