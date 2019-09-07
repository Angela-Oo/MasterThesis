#pragma once

#include "algo/registration/deformation_graph/deformation_graph.h"
#include "algo/registration/rigid_registration/rigid_deformation.h"
#include "algo/remeshing/mesh_simplification.h"
#include "mesh/mesh_definition.h"
#include "registration_type.h"
#include "util/ceres_include.h"

namespace Registration {

enum class AdaptiveRigidity
{
	REDUCE_RIGIDITY,
	RIGIDITY_COST
};

struct AdaptiveRigidityOptions
{
	bool enable{ false };
	AdaptiveRigidity adaptive_rigidity{ AdaptiveRigidity::RIGIDITY_COST };
	double smooth_cost_threshold{ 0.01 };
	double minimal_rigidity{ 0.1 };
};

struct RefinementOptions
{
	enum class Refinement
	{
		EDGE,
		VERTEX
	};

	bool enable{ false };
	double min_edge_length {0.05};
	unsigned int levels {4};
	Refinement refine {Refinement::VERTEX};
	double smooth_cost_threshold{ 0.05 };
};



struct SequenceRegistrationOptions
{
	bool enable;
	// if true init rigid deformation with non rigid global deformation else init rigid deformation with previouse rigid deformation
	bool init_rigid_deformation_with_non_rigid_globale_deformation;
	bool use_previouse_frame_for_rigid_registration;

	SequenceRegistrationOptions()
		: enable(false)
		, init_rigid_deformation_with_non_rigid_globale_deformation(true)
		, use_previouse_frame_for_rigid_registration(true)
	{}
};

struct ICPOptions
{
	bool enable;
	double correspondence_max_distance; // initial distance for corresponding finding 
	double correspondence_max_angle_deviation; // 45 degree

	ICPOptions()
		: enable(true)
		, correspondence_max_distance(0.1)
		, correspondence_max_angle_deviation(45.)
	{}
};

struct DeformationGraphOptions
{
	double edge_length;
	unsigned int number_of_interpolation_neighbors;

	DeformationGraphOptions()
		: edge_length(0.3)
		, number_of_interpolation_neighbors(4)
	{}
};

struct Input
{
	std::string file_path;
	std::string file_name;	
	size_t start_index;
	std::string output_folder_name;
	std::string image_folder_name;
	int number_of_frames_to_load; // to load all frames set to -1

	Input()
		: file_path("../input_data/HaoLi/head/finalRegistration/")
		, file_name("meshOfFrame")
		, start_index(1)
		, output_folder_name("head")
		, image_folder_name("images")
		, number_of_frames_to_load(-1)
	{}
};

struct RegistrationOptions
{
	Input input_mesh_sequence;
	RegistrationType type;
	DeformationGraphOptions deformation_graph;
	ceres::Solver::Options ceres_options;
	double smooth;
	double fit;
	unsigned int max_iterations;
	double use_vertex_random_probability; // value between 0. and 1.
	bool ignore_border_vertices;
	bool evaluate_residuals;
	bool error_evaluation;
	ReduceMeshStrategy mesh_reduce_strategy;

	bool rigid_and_non_rigid_registration;
	SequenceRegistrationOptions sequence_options;
	RefinementOptions refinement;
	AdaptiveRigidityOptions adaptive_rigidity;
	ICPOptions icp;

	RegistrationOptions()
		: type(RegistrationType::ARAP)
		, smooth(10.)
		, fit(10.)
		, max_iterations(25)
		, ignore_border_vertices(true)
		, evaluate_residuals(true)
		, error_evaluation(true)
		, use_vertex_random_probability(0.5)
		, mesh_reduce_strategy(ReduceMeshStrategy::ISOTROPIC)
	{}
};


}