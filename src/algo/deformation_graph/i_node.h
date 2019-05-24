#pragma once

#include "stdafx.h"
#include "mLibInclude.h"

// mesh.add_property_map<vertex_descriptor, std::shared_ptr<INode>>("node", nullptr);
class INode
{
public:
	virtual ml::vec3f & position() = 0;
	virtual ml::vec3d & normal() = 0;
	virtual double * r() = 0;
	virtual double * t() = 0;
	virtual double * w() = 0;
public:
	virtual ml::mat3d rotation() const = 0;
	virtual ml::vec3d translation() const = 0;
	virtual double weight() const = 0;
public:
	virtual ml::vec3d deformedPosition() const = 0;
	virtual ml::vec3d deformedNormal() const = 0;
	virtual ml::vec3d deformPosition(const ml::vec3f & pos) const = 0;
	virtual ml::vec3d deformNormal(const ml::vec3f & normal) const = 0;
};

//auto property_map = mesh.add_property_map<edge_descriptor, std::shared_ptr<IEdge>>("edge", nullptr);
class IEdge
{
public:
	//virtual Kernel::Point_3 deformed() = 0;
};