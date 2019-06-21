#include "stdafx.h"
#include "position_and_deformation.h"

namespace DG {


Point PositionAndDeformation::getDeformedPosition() const
{
	Vector v = _point - CGAL::ORIGIN;
	return CGAL::ORIGIN + v + _deformation->translation();
}

Vector PositionAndDeformation::getDeformedNormal() const
{
	return _deformation->rotation()(_normal);
}

Point PositionAndDeformation::deformPosition(Point point) const
{
	Vector rotated_point = _deformation->rotation()(point - _point);
	Vector moved_position = (_point - CGAL::ORIGIN) + _deformation->translation();
	return CGAL::ORIGIN + moved_position + rotated_point;
}

Vector PositionAndDeformation::deformNormal(Vector normal) const
{
	return _deformation->rotation()(normal);
}

}