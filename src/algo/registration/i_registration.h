#pragma once

#include "mLibInclude.h"
//#include "graph_node_type.h"

#include "algo/deformation_graph/deformation_graph_cgal_mesh.h"

struct Edge
{
	ml::vec3f source_point;
	ml::vec3f target_point;
	double cost = 0.;
};


class IRegistration
{
public:
	virtual bool finished() = 0;
	virtual bool solveIteration() = 0;
	virtual bool solve() = 0;
public:
	virtual const SurfaceMesh & getSource() = 0;
	virtual const SurfaceMesh & getTarget() = 0;
	virtual SurfaceMesh getDeformedPoints() = 0;
	//virtual SurfaceMesh getInverseDeformedPoints() = 0;
public:
	virtual std::vector<Point> getFixedPostions() { return std::vector<Point>(); }
	virtual const DG::DeformationGraphCgalMesh & getDeformationGraph() { return DG::DeformationGraphCgalMesh();	};
	virtual SurfaceMesh getDeformationGraphMesh() = 0;// { return SurfaceMesh(); };
public:
	virtual ~IRegistration() = default;
};

