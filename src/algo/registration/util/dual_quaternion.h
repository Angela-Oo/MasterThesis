#pragma once

#include "mLibCore.h"

/*dual quaternion class for CUDA kernel*/
struct Quaternion
{
	Quaternion() {}
	Quaternion(double _w, double _x, double _y, double _z) : q0(ml::vec4d(_x, _y, _z, _w)) {}
	Quaternion(const ml::vec4d &_q) : q0(_q) {}
	Quaternion(const ml::mat3d &_rot)
	{
		double tr = _rot(0, 0)+_rot(1, 1)+_rot(2, 2);
		if (tr > 0) {
			double s = sqrtf(tr + 1.0f) * 2;
			q0.w = s*0.25f;
			q0.x = (_rot(2, 1) - _rot(1, 2))/s;
			q0.y = (_rot(0, 2) - _rot(2, 0))/s;
			q0.z = (_rot(1, 0) - _rot(0, 1))/s;
		}
		else if ((_rot(0, 0) > _rot(1, 1)) && (_rot(0, 0) > _rot(2, 2))) {
			double s = sqrtf(1.0f+_rot(0, 0)-_rot(1, 1)-_rot(2, 2))*2;
			q0.w = (_rot(2, 1) - _rot(1, 2))/s;
			q0.x = 0.25f*s;
			q0.y = (_rot(0, 1) + _rot(1, 0))/s;
			q0.z = (_rot(0, 2) + _rot(2, 0))/s;
		}
		else if (_rot(1, 1) > _rot(2, 2)) {
			double s = sqrtf(1.0f+_rot(1, 1)-_rot(0, 0)-_rot(2, 2))*2;
			q0.w = (_rot(0, 2) - _rot(2, 0))/s;
			q0.x = (_rot(0, 1) + _rot(1, 0))/s;
			q0.y = 0.25f*s;
			q0.z = (_rot(1, 2) + _rot(2, 1))/s;
		}
		else {
			double s = sqrtf(1.0f+_rot(2, 2)-_rot(0, 0)-_rot(1, 1))*2;
			q0.w = (_rot(1, 0) - _rot(0, 1))/s;
			q0.x = (_rot(0, 2) + _rot(2, 0))/s;
			q0.y = (_rot(1, 2) + _rot(2, 1))/s;
			q0.z = 0.25f*s;
		}
	}

	double& x() { return q0.x; }
	double& y() { return q0.y; }
	double& z() { return q0.z; }
	double& w() { return q0.w; }

	const double& x() const { return q0.x; }
	const double& y() const { return q0.y; }
	const double& z() const { return q0.z; }
	const double& w() const { return q0.w; }
	
	Quaternion conjugate() const { return Quaternion(q0.w, -q0.x, -q0.y, -q0.z); }
	double square_norm() const { return q0.w*q0.w + q0.x*q0.x + q0.y*q0.y + q0.z*q0.z; }
	double norm() const { return sqrtf(square_norm()); }
	double dot(const Quaternion &_quat) const { return q0.w*_quat.w() + q0.x*_quat.x() + q0.y*_quat.y() + q0.z*_quat.z(); }
	void normalize() { q0 = q0.getNormalized(); }
	Quaternion normalized() const { Quaternion q(*this); q.normalize(); return q; }

	ml::mat3d matrix() const
	{
		/*normalize quaternion before converting to so3 matrix*/
		Quaternion q(*this);
		q.normalize();

		ml::mat3d rot;
		rot(0, 0) = 1-2*q.y()*q.y()-2*q.z()*q.z();
		rot(0, 1) = 2*q.x()*q.y()-2*q.z()*q.w();
		rot(0, 2) = 2*q.x()*q.z()+2*q.y()*q.w();
		rot(1, 0) = 2*q.x()*q.y()+2*q.z()*q.w();
		rot(1, 1) = 1-2*q.x()*q.x()-2*q.z()*q.z();
		rot(1, 2) = 2*q.y()*q.z()-2*q.x()*q.w();
		rot(2, 0) = 2*q.x()*q.z()-2*q.y()*q.w();
		rot(2, 1) = 2*q.y()*q.z()+2*q.x()*q.w();
		rot(2, 2) = 1-2*q.x()*q.x()-2*q.y()*q.y();
		return rot;
	}

	ml::vec3d vec() const { return ml::vec3d(q0.x, q0.y, q0.z); }

	ml::vec4d q0;
};

Quaternion operator+(const Quaternion &_left, const Quaternion &_right);

Quaternion operator*(double _scalar, const Quaternion &_quat);

Quaternion operator*(const Quaternion &_quat, double _scalar);

Quaternion operator*(const Quaternion &_q0, const Quaternion &_q1);

struct DualNumber {
	DualNumber() : q0(0), q1(0) {}
	DualNumber(double _q0, double _q1) : q0(_q0), q1(_q1) {}

	DualNumber operator+(const DualNumber &_dn) const
	{
		return{ q0+_dn.q0, q1+_dn.q1 };
	}

	DualNumber& operator+=(const DualNumber &_dn)
	{
		*this = *this + _dn;
		return *this;
	}

	DualNumber operator*(const DualNumber &_dn) const
	{
		return{ q0*_dn.q0, q0*_dn.q1 + q1*_dn.q0 };
	}

	DualNumber& operator*=(const DualNumber &_dn)
	{
		*this = *this * _dn;
		return *this;
	}

	DualNumber reciprocal() const
	{
		return{ 1.0f/q0, -q1/(q0*q0) };
	}

	DualNumber sqrt() const
	{
		return{ sqrtf(q0), q1/(2*sqrtf(q0)) };
	}

	double q0, q1;
};

// Forward declaration
struct DualQuaternion;
DualQuaternion operator*(const DualNumber &_dn, const DualQuaternion &_dq);

struct DualQuaternion {

	DualQuaternion() {}
	DualQuaternion(const Quaternion &_q0, const Quaternion &_q1) : q0(_q0), q1(_q1) {}
	DualQuaternion(const ml::mat4d & T) //mat34 &T)
	{
		ml::mat3d r = T.getRotation();
		ml::vec3d t = T.getTranslation();
		DualQuaternion rot_part(Quaternion(r), Quaternion(0, 0, 0, 0));
		DualQuaternion vec_part(Quaternion(1, 0, 0, 0), Quaternion(0, 0.5f*t.x, 0.5f*t.y, 0.5f*t.z));
		*this = vec_part * rot_part;
	}

	DualQuaternion operator+(const DualQuaternion &_dq) const
	{
		Quaternion quat0(q0+_dq.q0);
		Quaternion quat1(q1+_dq.q1);
		return{ quat0, quat1 };
	}

	DualQuaternion operator*(const DualQuaternion &_dq) const
	{
		Quaternion quat0(q0*_dq.q0);
		Quaternion quat1(q1*_dq.q0+q0*_dq.q1);
		return{ quat0, quat1 };
	}

	DualQuaternion& operator+=(const DualQuaternion &_dq)
	{
		*this = *this + _dq;
		return *this;
	}

	DualQuaternion& operator*=(const DualQuaternion &_dq)
	{
		*this = *this * _dq;
		return *this;
	}

	DualQuaternion operator*(const DualNumber &_dn) const
	{
		return _dn * *this;
	}

	DualQuaternion& operator*=(const DualNumber &_dn)
	{
		*this = *this * _dn;
		return *this;
	}

	operator DualNumber() const
	{
		return DualNumber(q0.w(), q1.w());
	}

	DualQuaternion conjugate() const
	{
		return{ q0.conjugate(), q1.conjugate() };
	}

	DualNumber squared_norm() const
	{
		return *this * this->conjugate();
	}

	DualNumber norm() const
	{
		float a0 = q0.norm();
		float a1 = q0.dot(q1)/q0.norm();
		return{ a0, a1 };
	}

	DualQuaternion inverse() const
	{
		return this->conjugate() * this->squared_norm().reciprocal();
	}

	void normalize()
	{
		*this = *this * this->norm().reciprocal();
	}

	DualQuaternion normalized() const
	{
		return *this * this->norm().reciprocal();
	}
	
	operator ml::mat4d() const//mat34() const
	{
		ml::mat3d r;
		ml::vec3d t;
		DualQuaternion quat_normalized = this->normalized();
		r = quat_normalized.q0.matrix();
		Quaternion vec_part = 2.0f*quat_normalized.q1*quat_normalized.q0.conjugate();
		t = vec_part.vec();
		ml::mat4d transformation;
		transformation.setRotationMatrix(r);
		transformation.setTranslationVector(t);
		return transformation;// mat34(r, t);
	}

	Quaternion q0, q1;
};
