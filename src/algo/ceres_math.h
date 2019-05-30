#pragma once

#include "../mLibInclude.h"
#include "algo/mesh_simplification/mesh_simplification.h"

template<typename T>
void vec3f_to_T(const ml::vec3f& v, T* vector)
{
	vector[0] = T(v[0]);
	vector[1] = T(v[1]);
	vector[2] = T(v[2]);
}

template<typename T>
void point_to_T(const Point& v, T* vector)
{
	vector[0] = T(v.x());
	vector[1] = T(v.y());
	vector[2] = T(v.z());
}

template<typename T>
void point_to_T(const Vector& n, T* vector)
{
	vector[0] = T(n.x());
	vector[1] = T(n.y());
	vector[2] = T(n.z());
}


template<typename T>
void mat3d_to_T(const ml::mat3d& m, T* matrix)
{
	matrix[0] = T(m(0, 0));
	matrix[1] = T(m(1, 0));
	matrix[2] = T(m(2, 0));
	matrix[3] = T(m(0, 1));
	matrix[4] = T(m(1, 1));
	matrix[5] = T(m(2, 1));
	matrix[6] = T(m(0, 2));
	matrix[7] = T(m(1, 2));
	matrix[8] = T(m(2, 1));
}

template<typename T>
void matrix_multiplication(const T * const m, const T * const v, T* result)
{
	T tmp[3];
	tmp[0] = m[0] * v[0] + m[1] * v[1] + m[2] * v[2];
	tmp[1] = m[3] * v[0] + m[4] * v[1] + m[5] * v[2];
	tmp[2] = m[6] * v[0] + m[7] * v[1] + m[8] * v[2];
	result[0] = tmp[0];
	result[1] = tmp[1];
	result[2] = tmp[2];
}

template<typename T>
void matrix_transpose(const T * const m, T * m_transpose)
{
	m_transpose[0] = m[0];
	m_transpose[4] = m[4];
	m_transpose[8] = m[8];

	m_transpose[1] = m[3];
	m_transpose[2] = m[6];

	m_transpose[3] = m[1];	
	m_transpose[5] = m[7];

	m_transpose[6] = m[2];
	m_transpose[7] = m[5];	
}



template<typename T>
void scalar_multiply(const T * const v, T scalar, T* result)
{
	result[0] = v[0] * scalar;
	result[1] = v[1] * scalar;
	result[2] = v[2] * scalar;
}

template<typename T>
void substract(const T * const v1, const T * const v2, T* result)
{
	result[0] = v1[0] - v2[0];
	result[1] = v1[1] - v2[1];
	result[2] = v1[2] - v2[2];
}

template<typename T>
void addition(const T * const v1, const T * const v2, T* result)
{
	result[0] = v1[0] + v2[0];
	result[1] = v1[1] + v2[1];
	result[2] = v1[2] + v2[2];
}

template<typename T>
T dot(const T * const v1, const T * const v2)
{
	T result = v1[0] * v2[0] + v1[1] * v2[1] + v1[2] * v2[2];
	return result;
}

template<typename T>
void normalize(const T * const v, T * normalized_v)
{
	T length = T(sqrt(dot(v, v)));
	T one_divided_by_length = T(1.) / length;
	if (length > T(0.))
		scalar_multiply<T>(v, one_divided_by_length, normalized_v);
	else {
		normalized_v[0] = T(0.);
		normalized_v[1] = T(0.);
		normalized_v[2] = T(0.);
	}
}

template<typename T>
T pow2(T v)
{
	return v * v;
}