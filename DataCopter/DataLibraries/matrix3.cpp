/*
 * matrix3.cpp
 * Copyright (C) Andrew Tridgell 2012
 *
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

//#pragma GCC optimize("O3")

#include "BIT_MATH.h"
//#include "iostream"
//#include <type_traits>
//#include <cassert>

// create a rotation matrix given some euler angles
// this is based on http://gentlenav.googlecode.com/files/EulerAngles.pdf
template <typename T>
void Matrix3<T>::from_euler(float roll, float pitch, float yaw)
{
    float cp = cosf(pitch);
    float sp = sinf(pitch);
    float sr = sinf(roll);
    float cr = cosf(roll);
    float sy = sinf(yaw);
    float cy = cosf(yaw);

    a.x = cp * cy;
    a.y = (sr * sp * cy) - (cr * sy);
    a.z = (cr * sp * cy) + (sr * sy);
    b.x = cp * sy;
    b.y = (sr * sp * sy) + (cr * cy);
    b.z = (cr * sp * sy) - (sr * cy);
    c.x = -sp;
    c.y = sr * cp;
    c.z = cr * cp;
}

// calculate euler angles from a rotation matrix
// this is based on http://gentlenav.googlecode.com/files/EulerAngles.pdf
template <typename T>
void Matrix3<T>::to_euler(float *roll, float *pitch, float *yaw) const
{
    if (pitch != NULL) {
        *pitch = -safe_asin(c.x);
    }
    if (roll != NULL) {
        *roll = atan2f(c.y, c.z);
    }
    if (yaw != NULL) {
        *yaw = atan2f(b.x, a.x);
    }
}

/*
  calculate Euler angles (312 convention) for the matrix.
  See http://www.atacolorado.com/eulersequences.doc
  vector is returned in r, p, y order
*/
template <typename T>
Vector3<T> Matrix3<T>::to_euler312() const
{
    return Vector3<T>(asinf(c.y),
                      atan2f(-c.x, c.z),
                      atan2f(-a.y, b.y));
}

/*
  fill the matrix from Euler angles in radians in 312 convention
*/
template <typename T>
void Matrix3<T>::from_euler312(float roll, float pitch, float yaw)
{
    float c3 = cosf(pitch);
    float s3 = sinf(pitch);
    float s2 = sinf(roll);
    float c2 = cosf(roll);
    float s1 = sinf(yaw);
    float c1 = cosf(yaw);

    a.x = c1 * c3 - s1 * s2 * s3;
    b.y = c1 * c2;
    c.z = c3 * c2;
    a.y = -c2*s1;
    a.z = s3*c1 + c3*s2*s1;
    b.x = c3*s1 + s3*s2*c1;
    b.z = s1*s3 - s2*c1*c3;
    c.x = -s3*c2;
    c.y = s2;
}

// apply an additional rotation from a body frame gyro vector
// to a rotation matrix.
template <typename T>
void Matrix3<T>::rotate(const Vector3<T> &g)
{
    Matrix3<T> temp_matrix;
    temp_matrix.a.x = a.y * g.z - a.z * g.y;
    temp_matrix.a.y = a.z * g.x - a.x * g.z;
    temp_matrix.a.z = a.x * g.y - a.y * g.x;
    temp_matrix.b.x = b.y * g.z - b.z * g.y;
    temp_matrix.b.y = b.z * g.x - b.x * g.z;
    temp_matrix.b.z = b.x * g.y - b.y * g.x;
    temp_matrix.c.x = c.y * g.z - c.z * g.y;
    temp_matrix.c.y = c.z * g.x - c.x * g.z;
    temp_matrix.c.z = c.x * g.y - c.y * g.x;

    (*this) += temp_matrix;
}

/*
  re-normalise a rotation matrix
*/
template <typename T>
void Matrix3<T>::normalize(void)
{
    float error = a * b;
    Vector3<T> t0 = a - (b * (0.5f * error));
    Vector3<T> t1 = b - (a * (0.5f * error));
    Vector3<T> t2 = t0 % t1;
    a = t0 * (1.0f / t0.length());
    b = t1 * (1.0f / t1.length());
    c = t2 * (1.0f / t2.length());
}
#if 0
// multiplication by a vector
template <typename T>
Vector3<T> Matrix3<T>::operator *(const Vector3<T> &v) const
{
    return Vector3<T>(a.x * v.x + a.y * v.y + a.z * v.z,
                      b.x * v.x + b.y * v.y + b.z * v.z,
                      c.x * v.x + c.y * v.y + c.z * v.z);
}
#endif
//
//// multiplication by a vector, extracting only the xy components
//template <typename T>
//Vector2<T> Matrix3<T>::mulXY(const Vector3<T> &v) const
//{
//    return Vector2<T>(a.x * v.x + a.y * v.y + a.z * v.z,
//                      b.x * v.x + b.y * v.y + b.z * v.z);
//}

// multiplication of transpose by a vector
//template <typename T>
//Vector3<T> Matrix3<T>::mul_transpose(const Vector3<T> &v) const
//{
//    return Vector3<T>(a.x * v.x + b.x * v.y + c.x * v.z,
//                      a.y * v.x + b.y * v.y + c.y * v.z,
//                      a.z * v.x + b.z * v.y + c.z * v.z);
//}
//
// multiplication by another Matrix3<T>
template <typename T>
Matrix3<T> Matrix3<T>::operator *(const Matrix3<T> &m) const
{
    Matrix3<T> temp (Vector3<T>(a.x * m.a.x + a.y * m.b.x + a.z * m.c.x,
                                a.x * m.a.y + a.y * m.b.y + a.z * m.c.y,
                                a.x * m.a.z + a.y * m.b.z + a.z * m.c.z),
                     Vector3<T>(b.x * m.a.x + b.y * m.b.x + b.z * m.c.x,
                                b.x * m.a.y + b.y * m.b.y + b.z * m.c.y,
                                b.x * m.a.z + b.y * m.b.z + b.z * m.c.z),
                     Vector3<T>(c.x * m.a.x + c.y * m.b.x + c.z * m.c.x,
                                c.x * m.a.y + c.y * m.b.y + c.z * m.c.y,
                                c.x * m.a.z + c.y * m.b.z + c.z * m.c.z));
    return temp;
}

template <typename T>
Matrix3<T> Matrix3<T>::transposed(void) const
{
    return Matrix3<T>(Vector3<T>(a.x, b.x, c.x),
                      Vector3<T>(a.y, b.y, c.y),
                      Vector3<T>(a.z, b.z, c.z));
}

template <typename T>
T Matrix3<T>::det() const
{
    return a.x * (b.y * c.z - b.z * c.y) +
           a.y * (b.z * c.x - b.x * c.z) +
           a.z * (b.x * c.y - b.y * c.x);
}




template <typename T>
bool Matrix3<T>::inverse(Matrix3<T>& inv) const
{
    T d = det();

//    if (is_zero(d)) {
//        return false;
//    }
    Vector3<T> temp(0.0,0.0,0.0);
    if(inv.a==temp&&inv.b==temp&&inv.c==temp)
    {
    	return false;
    }

    inv.a.x = (b.y * c.z - c.y * b.z) / d;
    inv.a.y = (a.z * c.y - a.y * c.z) / d;
    inv.a.z = (a.y * b.z - a.z * b.y) / d;
    inv.b.x = (b.z * c.x - b.x * c.z) / d;
    inv.b.y = (a.x * c.z - a.z * c.x) / d;
    inv.b.z = (b.x * a.z - a.x * b.z) / d;
    inv.c.x = (b.x * c.y - c.x * b.y) / d;
    inv.c.y = (c.x * a.y - a.x * c.y) / d;
    inv.c.z = (a.x * b.y - b.x * a.y) / d;

    return true;
}

template <typename T>
bool Matrix3<T>::invert()
{
    Matrix3<T> inv;
    bool success = inverse(inv);
    if (success) {
        *this = inv;
    }
    return success;
}

template <typename T>
void Matrix3<T>::zero(void)
{
    a.x = a.y = a.z = 0;
    b.x = b.y = b.z = 0;
    c.x = c.y = c.z = 0;
}

// create rotation matrix for rotation about the vector v by angle theta
// See: http://www.euclideanspace.com/maths/geometry/rotations/conversions/angleToMatrix/
template <typename T>
void Matrix3<T>::from_axis_angle(const Vector3<T> &v, float theta)
{
    float C = cosf(theta);
    float S = sinf(theta);
    float t = 1.0f - C;
    Vector3f normv = v.normalized();
    float x = normv.x;
    float y = normv.y;
    float z = normv.z;
    
    a.x = t*x*x + C;
    a.y = t*x*y - z*S;
    a.z = t*x*z + y*S;
    b.x = t*x*y + z*S;
    b.y = t*y*y + C;
    b.z = t*y*z - x*S;
    c.x = t*x*z - y*S;
    c.y = t*y*z + x*S;
    c.z = t*z*z + C;
}


// only define for float
template void Matrix3<float>::zero(void);
template void Matrix3<float>::rotate(const Vector3<float> &g);
template void Matrix3<float>::normalize(void);
template void Matrix3<float>::from_euler(float roll, float pitch, float yaw);
template void Matrix3<float>::to_euler(float *roll, float *pitch, float *yaw) const;
template void Matrix3<float>::from_euler312(float roll, float pitch, float yaw);
template void Matrix3<float>::from_axis_angle(const Vector3<float> &v, float theta);
template Vector3<float> Matrix3<float>::to_euler312(void) const;
template Vector3<float> Matrix3<float>::operator *(const Vector3<float> &v) const;
//template Vector3<float> Matrix3<float>::mul_transpose(const Vector3<float> &v) const;
template Matrix3<float> Matrix3<float>::operator *(const Matrix3<float> &m) const;
template Matrix3<float> Matrix3<float>::transposed(void) const;
template float Matrix3<float>::det() const;
template bool Matrix3<float>::inverse(Matrix3<float>& inv) const;
template bool Matrix3<float>::invert();
//template Vector2<float> Matrix3<float>::mulXY(const Vector3<float> &v) const;

template void Matrix3<double>::zero(void);
template void Matrix3<double>::rotate(const Vector3<double> &g);
template void Matrix3<double>::from_euler(float roll, float pitch, float yaw);
template void Matrix3<double>::to_euler(float *roll, float *pitch, float *yaw) const;
template Vector3<double> Matrix3<double>::operator *(const Vector3<double> &v) const;
//template Vector3<double> Matrix3<double>::mul_transpose(const Vector3<double> &v) const;
template Matrix3<double> Matrix3<double>::operator *(const Matrix3<double> &m) const;
template Matrix3<double> Matrix3<double>::transposed(void) const;
template double Matrix3<double>::det() const;
template bool Matrix3<double>::inverse(Matrix3<double>& inv) const;
template bool Matrix3<double>::invert();
//template Vector2<double> Matrix3<double>::mulXY(const Vector3<double> &v) const;
