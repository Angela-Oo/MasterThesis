#include "node.h"
#include <ceres/rotation.h>


ml::vec3d Node::deformedPosition() const
{
	return _g + _t;
}

ml::vec3d Node::deformedNormal() const
{
	auto r_t = rotation().getTranspose();
	auto normal = r_t * _n;
	return normal.getNormalized();
}

ml::vec3d Node::deformPosition(const ml::vec3f & pos) const
{
	return (rotation()*(pos - _g)) + _g + _t;
}

Node::Node(const ml::vec3f & g, const ml::vec3d & n)
	: Node(g, n, ml::mat3d::identity(), ml::vec3d::origin)
{}

Node::Node(const ml::vec3f & g, const ml::vec3d & n, const ml::mat3d & r, const ml::vec3d & t)
	: _g(g)
	, _n(n)
	, _r(r)
	, _t(t)
	, _w(1.)
{}

Node::Node()
	: Node(ml::vec3f::origin, ml::vec3f::eZ)
{}