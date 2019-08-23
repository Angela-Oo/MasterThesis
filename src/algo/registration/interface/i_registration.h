#pragma once

#include "algo/registration/deformation_graph/deformation_graph.h"
#include "algo/registration/rigid_registration/rigid_deformation.h"
#include "algo/remeshing/mesh_simplification.h"
#include "mesh/mesh_definition.h"

namespace Registration {


struct DeformationGraphOptions
{
	double  edge_length;
	unsigned int levels; // only used for refinement
	unsigned int number_of_interpolation_neighbors;

	DeformationGraphOptions()
		: edge_length(0.05)
		, levels(4)
		, number_of_interpolation_neighbors(4)
	{}
};

struct SequenceRegistrationOptions
{
	// if true init rigid deformation with non rigid global deformation else init rigid deformation with previouse rigid deformation
	bool init_rigid_deformation_with_non_rigid_globale_deformation;
	bool use_previouse_frame_for_rigid_registration;

	SequenceRegistrationOptions()
		: init_rigid_deformation_with_non_rigid_globale_deformation(true)
		, use_previouse_frame_for_rigid_registration(true)
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
	SequenceRegistrationOptions sequence_options;
	DeformationGraphOptions dg_options;
	ReduceMeshStrategy mesh_reduce_strategy;

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
		, mesh_reduce_strategy(ReduceMeshStrategy::ISOTROPIC)
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
	virtual ~IRegistration() = default;
};



class INonRigidRegistration : public IRegistration
{
public:
	virtual void setRigidDeformation(const RigidDeformation & rigid_deformation) = 0;
	virtual std::vector<Point> getFixedPostions() { return std::vector<Point>(); }
	virtual SurfaceMesh getDeformationGraphMesh() = 0;
	virtual bool shouldBeSavedAsImage() = 0;
public:
	virtual ~INonRigidRegistration() = default;
};



}