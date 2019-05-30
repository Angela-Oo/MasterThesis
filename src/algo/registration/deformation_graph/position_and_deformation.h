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
	Direction _normal;
public:
	Point getDeformedPosition() const;
	Direction getDeformedNormal() const;

	Point deformPosition(Point point) const;
	Direction deformNormal(Direction normal) const;
};

}