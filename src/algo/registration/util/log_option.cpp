#include "log_option.h"

namespace Registration {

void logRegistrationOptions(std::shared_ptr<FileWriter> logger, const RegistrationOptions & registration_options)
{
	std::stringstream ss;
	ss << std::endl << "Registration Options: " << std::endl
		<< " cost function weights: " << " smooth: " << registration_options.smooth
		<< ", fit: " << registration_options.fit
		<< ", conf: " << registration_options.conf << std::endl
		<< " deformation graph " << "edge length: " << registration_options.dg_options.edge_length << std::endl
		<< " number of interpolated neighbors (k): " << registration_options.dg_options.number_of_interpolation_neighbors << std::endl
		<< " max iterations: " << registration_options.max_iterations << std::endl
		<< " ignore border vertices: " << std::boolalpha << registration_options.ignore_deformation_graph_border_vertices << std::endl
		<< " random probability to use a corresponding vertex: " << registration_options.use_vertex_random_probability << std::endl
		<< " correspondence finding" << " max distance: " << registration_options.correspondence_max_distance
		<< ", max angle: " << registration_options.correspondence_max_angle_deviation << std::endl
		<< " evaluate residuals: " << std::boolalpha << registration_options.evaluate_residuals << std::endl;
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

}