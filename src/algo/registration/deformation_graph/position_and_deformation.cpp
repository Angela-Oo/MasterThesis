#include "stdafx.h"
#include "position_and_deformation.h"

namespace Registration {


Point PositionAndDeformation::getDeformedPosition() const
{
	Vector v = _point - CGAL::ORIGIN;
	return CGAL::ORIGIN + v + _deformation->translation();
}


Point PositionAndDeformation::deformPosition(Point point) const
{
	auto edge = point - _point;
	Vector rotated_point = _deformation->rotation()(edge);
	Vector moved_position = (_point - CGAL::ORIGIN) + _deformation->translation();
	return CGAL::ORIGIN + moved_position + rotated_point;
}

Vector PositionAndDeformation::deformNormal(Vector normal) const
{
	return _deformation->rotation()(normal);
}

}