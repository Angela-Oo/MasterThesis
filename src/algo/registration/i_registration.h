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


struct DeformationGraphOptions
{
	double  edge_length;
	unsigned int number_of_interpolation_neighbors;

	DeformationGraphOptions()
		: edge_length(0.05)
		, number_of_interpolation_neighbors(4)
	{}
};

struct RegistrationOptions
{
	double smooth;
	double conf;
	double fit;
	double correspondence_max_distance; // initial distance for corresponding finding 
	double correspondence_max_angle_deviation; // 45 degree
	unsigned int max_iterations;
	bool ignore_deformation_graph_border_vertices;
	bool evaluate_residuals;
	double use_vertex_random_probability; // value between 0. and 1.
	DeformationGraphOptions dg_options;
	
	RegistrationOptions()
		: smooth(10.)
		, conf(10.)
		, fit(10.)
		, correspondence_max_distance(0.1)
		, correspondence_max_angle_deviation(45.)
		, max_iterations(25)
		, ignore_deformation_graph_border_vertices(true)
		, evaluate_residuals(true)
		, use_vertex_random_probability(0.5)
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
	virtual SurfaceMesh getDeformationGraphMesh() = 0;
public:
	virtual ~IRegistration() = default;
};

