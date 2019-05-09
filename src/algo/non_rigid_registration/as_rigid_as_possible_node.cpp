#include "as_rigid_as_possible_node.h"
#include <ceres/rotation.h>


const ml::mat3d & ARAPNode::rotation() const
{
	ml::mat3d r;	
	ceres::AngleAxisToRotationMatrix(_r.array, r.getData());
	return r.getTranspose(); // why??
}

ml::vec3d ARAPNode::deformedPosition() const
{
	return _g + _t;
}

ml::vec3d ARAPNode::deformedNormal() const
{
	auto r_t = rotation().getTranspose();
	auto normal = r_t * _n;
	return normal.getNormalized();
}

ml::vec3d ARAPNode::deformPosition(const ml::vec3f & pos) const
{
	auto r = rotation();
	return (r*(pos - _g)) + _g + _t;
}

ARAPNode::ARAPNode(int index, const ml::vec3f & g, const ml::vec3d & n)
	: ARAPNode(index, g, n, ml::vec3d::origin, ml::vec3d::origin)
{}

ARAPNode::ARAPNode(int index, const ml::vec3f & g, const ml::vec3d & n, const ml::vec3d & r, const ml::vec3d & t)
	: _index(index)
	, _g(g)
	, _n(n)
	, _r(r)
	, _t(t)
	, _w(1.)
{}

ARAPNode::ARAPNode()
	: ARAPNode(0, ml::vec3f::origin, ml::vec3f::eZ)
{}

ARAPNode::ARAPNode(const ARAPNode & node, bool inverse)
	: _index(node._index)
	, _g(node._g)
	, _n(node._n)
	, _r(node._r)
	, _t(node._t)
	, _w(node._w)
{
	if (inverse) {
		_g = node.deformedPosition();
		_n = node.deformedNormal();
		_r = -node._r;
		_t = -node._t;
	}
}
