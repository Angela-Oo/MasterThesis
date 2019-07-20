#pragma once

#include "mLibInclude.h"
#include "algo/surface_mesh/mesh_definition.h"
#include "i_deformation.h"

namespace DG
{

class PositionAndDeformation
{
public:
	std::shared_ptr<IDeformation> _deformation;
	Point _point;
	Vector _normal;
public:
	Point getDeformedPosition() const;
	Vector getDeformedNormal() const;

	Point deformPosition(Point point) const;
	Vector deformNormal(Vector normal) const;
	PositionAndDeformation()
		: _point(CGAL::ORIGIN)
		, _normal(0., 0., 0.)
	{}
};

}