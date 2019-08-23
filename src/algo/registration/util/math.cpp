#include "math.h"


Matrix convertMatrix(const ml::mat3d &m)
{
	Matrix matrix(m(0, 0), m(0, 1), m(0, 2),
				  m(1, 0), m(1, 1), m(1, 2),
				  m(2, 0), m(2, 1), m(2, 2));
	return matrix;
}

ml::mat3d convertMatrix(const Matrix &m)
{
	ml::mat3d matrix(m.m(0, 0), m.m(0, 1), m.m(0, 2),
					 m.m(1, 0), m.m(1, 1), m.m(1, 2),
					 m.m(2, 0), m.m(2, 1), m.m(2, 2));
	return matrix;
}

Vector convertVector(const ml::vec3d & v)
{
	return Vector(v[0], v[1], v[2]);
}

ml::vec3d convertVector(const Vector & v)
{
	return ml::vec3d(v[0], v[1], v[2]);
}