#pragma once

#include "../mLibInclude.h"


template<typename T>
void vec3f_to_T(const ml::vec3f& v, T* vector)
{
	vector[0] = T(v[0]);
	vector[1] = T(v[1]);
	vector[2] = T(v[2]);
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
void multiply(const T * const m, const T * const v, T* result)
{
	T res[3];
	res[0] = m[0] * v[0] + m[1] * v[1] + m[2] * v[2];
	res[1] = m[3] * v[0] + m[4] * v[1] + m[5] * v[2];
	res[2] = m[6] * v[0] + m[7] * v[1] + m[8] * v[2];
	result[0] = res[0];
	result[1] = res[1];
	result[2] = res[2];
}

template<typename T>
T dot(const T * const v1, const T * const v2)
{
	T result = v1[0] * v2[0] + v1[1] * v2[1] + v1[2] * v2[2];
	return result;
}

template<typename T>
T pow2(T v)
{
	return v * v;
}