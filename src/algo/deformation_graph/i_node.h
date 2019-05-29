#pragma once

#include "stdafx.h"
#include "algo/mesh_simplification/deformation_graph_mesh.h"

// mesh.add_property_map<vertex_descriptor, std::shared_ptr<INode>>("v:node", nullptr);
class INode
{
public:
	//virtual Point & position() = 0;
	//virtual Point & normal() = 0;
	virtual double * r() = 0;
	virtual double * t() = 0;
	virtual double * w() = 0;
public:
	virtual Matrix rotation() const = 0;
	virtual Vector translation() const = 0;
	virtual double weight() const = 0;
};

//auto property_map = mesh.add_property_map<edge_descriptor, std::shared_ptr<IEdge>>("edge", nullptr);
class IEdge
{
public:
	//virtual Kernel::Point_3 deformed() = 0;
};