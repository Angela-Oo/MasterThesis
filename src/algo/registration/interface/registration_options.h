#pragma once

#include "algo/registration/deformation_graph/deformation_graph.h"
#include "algo/remeshing/mesh_simplification.h"
#include "registration_type.h"
#include "util/ceres_include.h"
#include "visualize/visualizer/i_registration_visualizer.h"

namespace Registration {


struct AdaptiveRigidityOptions
{
	bool enable{ false };
	double rigidity_cost_coefficient{ 0.005 };
};

struct ReduceRigidityOptions
{
	bool enable{ false };
	double rigidity_cost_threshold{ 0.01 };
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
	bool use_previous_frame_for_rigid_registration;

	SequenceRegistrationOptions()
		: enable(false)
		, init_rigid_deformation_with_non_rigid_globale_deformation(true)
		, use_previous_frame_for_rigid_registration(true)
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
	double edge_length {0.3};
	unsigned int number_of_interpolation_neighbors{4};
};

struct Input
{
	std::string file_path{ "../input_data/HaoLi/head/finalRegistration/" };
	std::string file_name { "meshOfFrame" };
	size_t start_index{ 1 };
	std::string output_folder_name{ "head" };
	std::string image_folder_name{ "images" };
	int number_of_frames_to_load{ -1 }; // to load all frames set to -1
	Visualizer::RegistrationRenderMode render_mode{ Visualizer::RegistrationRenderMode::DEFORMATION };
	bool term{ true };
};

struct RegistrationOptions
{
	Input input_mesh_sequence;
	
	RegistrationType type{ RegistrationType::ARAP };
	DeformationGraphOptions deformation_graph;
	ceres::Solver::Options ceres_options;
	
	double smooth { 1. };
	double fit { 1. };
	unsigned int max_iterations{ 25 };
	double use_vertex_random_probability{ 0.2 }; // value between 0. and 1.
	bool ignore_border_vertices{ true };
	bool evaluate_residuals{ true };
	bool error_evaluation{ true };
	ReduceMeshStrategy mesh_reduce_strategy{ ReduceMeshStrategy::ISOTROPIC };
	bool rigid_and_non_rigid_registration{ true };
	
	SequenceRegistrationOptions sequence_options;
	RefinementOptions refinement;
	AdaptiveRigidityOptions adaptive_rigidity;
	ReduceRigidityOptions reduce_rigidity;
	ICPOptions icp;
};


}
