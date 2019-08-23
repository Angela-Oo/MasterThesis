#include "dual_quaternion.h"

Quaternion operator+(const Quaternion &_left, const Quaternion &_right)
{
	return{ _left.w()+_right.w(), _left.x()+_right.x(), _left.y()+_right.y(), _left.z()+_right.z() };
}

Quaternion operator*(double _scalar, const Quaternion &_quat)
{
	return{ _scalar*_quat.w(), _scalar*_quat.x(), _scalar*_quat.y(), _scalar*_quat.z() };
}

Quaternion operator*(const Quaternion &_quat, double _scalar)
{
	return _scalar * _quat;
}

Quaternion operator*(const Quaternion &_q0, const Quaternion &_q1)
{
	Quaternion q;
	q.w() = _q0.w()*_q1.w() - _q0.x()*_q1.x() - _q0.y()*_q1.y() - _q0.z()*_q1.z();
	q.x() = _q0.w()*_q1.x() + _q0.x()*_q1.w() + _q0.y()*_q1.z() - _q0.z()*_q1.y();
	q.y() = _q0.w()*_q1.y() - _q0.x()*_q1.z() + _q0.y()*_q1.w() + _q0.z()*_q1.x();
	q.z() = _q0.w()*_q1.z() + _q0.x()*_q1.y() - _q0.y()*_q1.x() + _q0.z()*_q1.w();

	return q;
}

DualQuaternion operator*(const DualNumber &_dn, const DualQuaternion &_dq)
{
	Quaternion quat0 = _dn.q0*_dq.q0;
	Quaternion quat1 = _dn.q0*_dq.q1+_dn.q1*_dq.q0;
	return{ quat0, quat1 };
}
