/**
 * @file data_type.h
 * @brief Defines all data types used in this lib

 * Mostly alias from Eigen Library.
 */

#include <stdio.h>
#include <math.h>
#include <limits>
#include <vector>
#include <Eigen/Geometry>
#include <Eigen/StdVector>

///Set red font in printf funtion
#ifndef ANSI_COLOR_RED
#define ANSI_COLOR_RED "\x1b[1;31m"
#endif
///Set green font in printf funtion
#ifndef ANSI_COLOR_GREEN
#define ANSI_COLOR_GREEN "\x1b[1;32m"
#endif
///Set yellow font in printf funtion
#ifndef ANSI_COLOR_YELLOW
#define ANSI_COLOR_YELLOW "\x1b[1;33m"
#endif
///Set blue font in printf funtion
#ifndef ANSI_COLOR_BLUE
#define ANSI_COLOR_BLUE "\x1b[1;34m"
#endif
///Set magenta font in printf funtion
#ifndef ANSI_COLOR_MAGENTA
#define ANSI_COLOR_MAGENTA "\x1b[1;35m"
#endif
///Set cyan font in printf funtion
#ifndef ANSI_COLOR_CYAN
#define ANSI_COLOR_CYAN "\x1b[1;36m"
#endif
///Reset font color in printf funtion
#ifndef ANSI_COLOR_RESET
#define ANSI_COLOR_RESET "\x1b[0m"
#endif

#ifndef DATA_TYPE_H
#define DATA_TYPE_H
/*! \brief Rename the float type used in lib

    Default is set to be double, but user can change it to float.
*/
typedef double decimal_t;

///Pre-allocated std::vector for Eigen using vec_E
template <typename T>
using vec_E = std::vector<T, Eigen::aligned_allocator<T>>;
///Eigen 1D float vector
template <int N>
using Vecf = Eigen::Matrix<decimal_t, N, 1>;
///Eigen 1D int vector
template <int N>
using Veci = Eigen::Matrix<int, N, 1>;
///MxN Eigen matrix
template <int M, int N>
using Matf = Eigen::Matrix<decimal_t, M, N>;
///MxN Eigen matrix with M unknown
template <int N>
using MatDNf = Eigen::Matrix<decimal_t, Eigen::Dynamic, N>;
///Vector of Eigen 1D float vector
template <int N>
using vec_Vecf = vec_E<Vecf<N>>;
///Vector of Eigen 1D int vector
template <int N>
using vec_Veci = vec_E<Veci<N>>;

///Eigen 1D float vector of size 2
typedef Vecf<2> Vec2f;
///Eigen 1D int vector of size 2
typedef Veci<2> Vec2i;
///Eigen 1D float vector of size 3
typedef Vecf<3> Vec3f;
///Eigen 1D int vector of size 3
typedef Veci<3> Vec3i;
///Eigen 1D float vector of size 4
typedef Vecf<4> Vec4f;
///Column vector in float of size 6
typedef Vecf<6> Vec6f;

///Vector of type Vec2f.
typedef vec_E<Vec2f> vec_Vec2f;
///Vector of type Vec2i.
typedef vec_E<Vec2i> vec_Vec2i;
///Vector of type Vec3f.
typedef vec_E<Vec3f> vec_Vec3f;
///Vector of type Vec3i.
typedef vec_E<Vec3i> vec_Vec3i;

///2x2 Matrix in float
typedef Matf<2, 2> Mat2f;
///3x3 Matrix in float
typedef Matf<3, 3> Mat3f;
///4x4 Matrix in float
typedef Matf<4, 4> Mat4f;
///6x6 Matrix in float
typedef Matf<6, 6> Mat6f;

///Dynamic Nx1 Eigen float vector
typedef Vecf<Eigen::Dynamic> VecDf;
///Nx2 Eigen float matrix
typedef MatDNf<2> MatD2f;
///Nx3 Eigen float matrix
typedef MatDNf<3> MatD3f;
///Dynamic MxN Eigen float matrix
typedef Matf<Eigen::Dynamic, Eigen::Dynamic> MatDf;

///Allias of Eigen::Affine2d
typedef Eigen::Transform<decimal_t, 2, Eigen::Affine> Aff2f;
///Allias of Eigen::Affine3d
typedef Eigen::Transform<decimal_t, 3, Eigen::Affine> Aff3f;
#endif

#ifndef EIGEN_QUAT
#define EIGEN_QUAT
///Allias of Eigen::Quaterniond
typedef Eigen::Quaternion<decimal_t> Quatf;
#endif

#ifndef EIGEN_EPSILON
#define EIGEN_EPSILON
///Compensate for numerical error
constexpr decimal_t epsilon_ = 1e-10; // numerical calculation error
#endif
