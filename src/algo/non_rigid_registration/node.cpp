#include "node.h"
#include <ceres/rotation.h>


ml::vec3f & Node::g() { return _g; };
ml::vec3d & Node::n() { return _n; };
double * Node::r() { return (&_r)->getData(); };
double * Node::t() { return (&_t)->getData(); };
double * Node::w() { return &_w; }

ml::vec3f Node::position() const
{ return _g; }

ml::vec3d Node::normal() const 
{ return _n; }

const ml::mat3d & Node::rotation() const
{
	return _r;
	//ml::mat3d r;
	//ceres::AngleAxisToRotationMatrix(_r.array, r.getData());
	//return r;
}

const ml::vec3d & Node::translation() const
{ return _t; }

double Node::weight() const
{ return _w; }

ml::vec3d Node::deformedPosition() const
{
	return _g + _t;
}

ml::vec3d Node::deformedNormal() const
{
	auto r_t = rotation().getTranspose();
	auto normal = r_t * _n;
	return normal.getNormalized();
	//matrix_multiplication(rotation_t, normal, normal);
	//matrix_multiplication(global_rotation_t, normal, normal);
}

ml::vec3d Node::deformPosition(const ml::vec3f & pos) const
{
	return (rotation()*(pos - _g)) + _g + _t;
}


Node::Node(const ml::vec3f & g, const ml::vec3d & n)
	: Node(g, n, ml::mat3d::identity(), ml::vec3d::origin)//	, _r(ml::mat3d::identity())
{}

//Node::Node(const ml::vec3f & g, const ml::vec3d & n, const ml::vec3d & r, const ml::vec3d & t)
//	: _g(g)
//	, _n(n)
//	, _r(ml::mat3d::identity())//r)
//	, _t(t)
//	, _w(1.)
//{}

Node::Node(const ml::vec3f & g, const ml::vec3d & n, const ml::mat3d & r, const ml::vec3d & t)
	: _g(g)
	, _n(n)
	, _r(r)//ml::vec3d::origin) // todo
	, _t(t)
	, _w(1.)
{}

Node::Node()
	: Node(ml::vec3f::origin, ml::vec3f::eZ)
{}