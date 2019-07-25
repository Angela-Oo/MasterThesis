#pragma once

#include "stdafx.h"
#include "algo/surface_mesh/mesh_definition.h"
#include "algo/registration/rigid_registration/rigid_deformation.h"


namespace Registration {

class IPositionDeformation
{
public:
	virtual Matrix rotation() const = 0;
	virtual Vector translation() const = 0;
	virtual Point position() const = 0;

	virtual Point getDeformedPosition() const = 0;
	virtual Point deformPosition(const Point & point) const = 0;
	virtual Vector deformNormal(const Vector & normal) const = 0;
};

}