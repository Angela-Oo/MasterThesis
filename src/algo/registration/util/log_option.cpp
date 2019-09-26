#include "log_option.h"

namespace Registration {

void logRegistrationOptions(std::shared_ptr<FileWriter> logger, const RegistrationOptions & options)
{
	std::stringstream ss;

	ss << std::endl << "Options: " << std::endl;

	ss << "Input " << options.input_mesh_sequence.output_folder_name
		<< ", file path " << options.input_mesh_sequence.file_path
		<< ", file name " << options.input_mesh_sequence.file_name
		<< ", number of frames to load: " << options.input_mesh_sequence.number_of_frames_to_load << std::endl << std::endl;

	if (options.sequence_options.enable) {
		ss << "Sequence registration: "
		<< " init rigid with non rigid deformation: " << std::boolalpha << options.sequence_options.init_rigid_deformation_with_non_rigid_globale_deformation
		<< ", use previous frame for rigid registration: " << std::boolalpha << options.sequence_options.use_previous_frame_for_rigid_registration << std::endl;
	}

	ss << "Registration Type: ";
	if (options.type == RegistrationType::ARAP)
		ss << "ARAP";
	else if (options.type == RegistrationType::ED)
		ss << "Embedded Deformation";
	else if (options.type == RegistrationType::Rigid)
		ss << "Rigid";
	else
		ss << "other type";
	ss << std::endl;
	
	ss << "Deformation Graph: ";
	ss << "edge length: " << options.deformation_graph.edge_length << std::endl;
	
	if (options.rigid_and_non_rigid_registration)
		ss << "Rigid and non Rigid registration " << std::endl;

	if(options.refinement.enable)
	{
		ss << "Deformation Graph Refinement: ";
		if (options.refinement.refine == Refinement::EDGE)
			ss << "refine at edge,";
		else
			ss << "refine at vertex,";
		ss << " smooth cost threshold: max(" << options.refinement.smooth_cost_threshold
			<< ", " << options.refinement.smooth_cost_percentage_of_max << " * max_value)"
			<< ", levels " << options.refinement.levels
			<< ", min edge length " << options.refinement.min_edge_length << std::endl;
	}
	if (options.adaptive_rigidity.enable) {
		ss << "Adaptive Rigidity by cost function:"
			<< " rigidity cost coefficient: " << options.adaptive_rigidity.rigidity_cost_coefficient
			<< ", minimal rigidity weight: " << options.adaptive_rigidity.minimal_rigidity_weight;

		if(options.adaptive_rigidity.refinement == Refinement::EDGE)
			ss << ", edge " << std::endl;
		else
			ss << ", vertex " << std::endl;
		if (options.adaptive_rigidity.regular == AdaptiveRigidityRegularizer::LINEAR)
			ss << ", linear regularization " << std::endl;
		else
			ss << ", quadratic regularization " << std::endl;
	}

	if (options.reduce_rigidity.enable)
	{
		ss << "Adaptive Rigidity by reducing"
			<< ", minimal rigidity: " << options.reduce_rigidity.minimal_rigidity
			<< ", rigidity cost threshold: " << options.reduce_rigidity.rigidity_cost_threshold << std::endl;
	}
	
	ss << std::endl;
	ss << "Max iterations: " << options.max_iterations << std::endl;
	ss << "Random probability to use a vertex: " << options.use_vertex_random_probability << std::endl;
	
	ss << "Smooth cost weight: " << options.smooth << " Fit cost weight: " << options.fit << std::endl;
	ss << "Reduce smooth factor: " << std::boolalpha << options.reduce_smooth_factor << std::endl;
	ss << "Ignore border vertices: " << std::boolalpha << options.ignore_border_vertices << std::endl;

	ss << "Correspondence find criteria: " << " max distance: " << options.icp.correspondence_max_distance
		<< ", max angle: " << options.icp.correspondence_max_angle_deviation << std::endl;
	
	logger->write(ss.str());
}

void logCeresOptions(std::shared_ptr<FileWriter> logger, const ceres::Solver::Options & ceres_options)
{
	std::map<ceres::LinearSolverType, std::string> linear_solver_type_map{
		{ceres::LinearSolverType::CGNR, "cgnr"},
		{ceres::LinearSolverType::DENSE_NORMAL_CHOLESKY, "dense normal choleski"},
		{ceres::LinearSolverType::DENSE_QR, "dense qr"},
		{ceres::LinearSolverType::DENSE_SCHUR, "dense schur"},
		{ceres::LinearSolverType::ITERATIVE_SCHUR, "iterative schur"},
		{ceres::LinearSolverType::SPARSE_NORMAL_CHOLESKY, "sparse normal choleski"},
		{ceres::LinearSolverType::SPARSE_SCHUR, "sparse schur"} };
	
	std::map<ceres::PreconditionerType, std::string> preconditioner_type_map{
		{ceres::PreconditionerType::JACOBI, "jacobi"},
		{ceres::PreconditionerType::IDENTITY, "identity"},
		{ceres::PreconditionerType::SCHUR_JACOBI, "schur jacobi"},
		{ceres::PreconditionerType::CLUSTER_JACOBI, "cluster jacobi"},
		{ceres::PreconditionerType::CLUSTER_TRIDIAGONAL, "cluster tridiagonal"} };


	std::map<ceres::TrustRegionStrategyType, std::string> trust_region_strategy_type_map{
		{ceres::TrustRegionStrategyType::DOGLEG, "dogleg"},
		{ceres::TrustRegionStrategyType::LEVENBERG_MARQUARDT, "levenberg marquardt"} };


	std::map<ceres::LineSearchDirectionType, std::string> line_search_direction_type_map{
		{ceres::LineSearchDirectionType::BFGS, "bfgs"},
		{ceres::LineSearchDirectionType::LBFGS, "lbfgs"},
		{ceres::LineSearchDirectionType::NONLINEAR_CONJUGATE_GRADIENT, "nonlinear conjugate gradient"}, 
		{ceres::LineSearchDirectionType::STEEPEST_DESCENT, "steepest descent"} };

	std::map<ceres::SparseLinearAlgebraLibraryType, std::string> sparse_linear_algebra_library_type_map {
		{ceres::SparseLinearAlgebraLibraryType::EIGEN_SPARSE, "cgnr"},
		{ceres::SparseLinearAlgebraLibraryType::NO_SPARSE, "no sparse"},
		{ceres::SparseLinearAlgebraLibraryType::SUITE_SPARSE, "suite sparse"},
		{ceres::SparseLinearAlgebraLibraryType::CX_SPARSE, "cx sparse"}
	};

	std::stringstream ss;
	ss << std::endl << "Ceres Solver:" << std::endl
		<< " Ceres sparse linear solver type: " << ceres_options.linear_solver_type << std::endl
		<< " Ceres preconditioner type: " << preconditioner_type_map[ceres_options.preconditioner_type] << std::endl
		<< " Ceres trust region strategy type: " << trust_region_strategy_type_map[ceres_options.trust_region_strategy_type] << std::endl
		<< " Ceres line search direction type: " << line_search_direction_type_map[ceres_options.line_search_direction_type] << std::endl
		<< " Ceres sparse linear algebra library type: " << sparse_linear_algebra_library_type_map[ceres_options.sparse_linear_algebra_library_type] << std::endl
		<< " Ceres number threads: " << ceres_options.num_threads << std::endl
		<< " Ceres max number iterations: " << ceres_options.max_num_iterations << std::endl;
		
	logger->write(ss.str());
}


void logOptions(std::shared_ptr<FileWriter> logger, const RegistrationOptions & registration_options)
{
	logRegistrationOptions(logger, registration_options);
	logCeresOptions(logger, registration_options.ceres_options);
}

}