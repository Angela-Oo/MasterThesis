#pragma once

#include "stdafx.h"
#include "algo/surface_mesh/mesh_definition.h"


// mesh.add_property_map<vertex_descriptor, std::shared_ptr<IDeformation>>("v:node", nullptr);
class IDeformation
{
public:
	virtual double * r() = 0;
	virtual double * t() = 0;
	virtual double * w() = 0;
public:
	virtual Matrix rotation() const = 0;
	virtual Vector translation() const = 0;
	virtual double weight() const = 0;
	virtual std::shared_ptr<IDeformation> invertDeformation() const = 0;
};

