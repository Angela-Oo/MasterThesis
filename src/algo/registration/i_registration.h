#pragma once

#include "mLibInclude.h"
//#include "graph_node_type.h"

#include "algo/registration/deformation_graph/deformation_graph.h"

struct Edge
{
	ml::vec3f source_point;
	ml::vec3f target_point;
	double cost = 0.;
};


struct RegistrationOptions
{
	double smooth;
	double conf;
	double fit;
	double initial_max_correspondence_distance;
	double max_correspondence_angle;
	unsigned int max_iterations;
	bool ignore_deformation_graph_border_vertices;
	
	RegistrationOptions()
		: smooth(5.)
		, conf(0.02)
		, fit(20.)
		, initial_max_correspondence_distance(0.1)
		, max_correspondence_angle(45.)
		, max_iterations(25)
		, ignore_deformation_graph_border_vertices(false)
	{}
};

class IRegistration
{
public:
	virtual bool finished() = 0;
	virtual bool solveIteration() = 0;
	virtual size_t currentIteration() = 0;
	virtual bool solve() = 0;
public:
	virtual const SurfaceMesh & getSource() = 0;
	virtual const SurfaceMesh & getTarget() = 0;
	virtual SurfaceMesh getDeformedPoints() = 0;
	virtual SurfaceMesh getInverseDeformedPoints() = 0;
public:
	virtual std::vector<Point> getFixedPostions() { return std::vector<Point>(); }
	virtual const DG::DeformationGraph & getDeformationGraph() { return DG::DeformationGraph();	}; // todo
	virtual SurfaceMesh getDeformationGraphMesh() = 0;// { return SurfaceMesh(); };
public:
	virtual ~IRegistration() = default;
};

