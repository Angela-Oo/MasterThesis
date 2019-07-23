#pragma once

#include "mLibInclude.h"
#include "algo/surface_mesh/mesh_definition.h"
#include "i_deformation.h"

namespace Registration {

class PositionAndDeformation
{
public:
	std::shared_ptr<IPositionDeformation> _deformation;
	Point _point;
public:
	Point getDeformedPosition() const;

	Point deformPosition(Point point) const;
	Vector deformNormal(Vector normal) const;
	PositionAndDeformation()
		: _point(CGAL::ORIGIN)
	{}
};

}