#include "ceres_option.h"

ceres::Solver::Options ceresOption() {
	ceres::Solver::Options options;
	options.sparse_linear_algebra_library_type = ceres::SparseLinearAlgebraLibraryType::SUITE_SPARSE;// ceres::EIGEN_SPARSE;
	options.minimizer_type = ceres::MinimizerType::TRUST_REGION;
	options.trust_region_strategy_type = ceres::TrustRegionStrategyType::LEVENBERG_MARQUARDT;
	options.line_search_direction_type = ceres::LineSearchDirectionType::LBFGS;
	options.linear_solver_type = ceres::LinearSolverType::SPARSE_SCHUR; //DENSE_SCHUR; //ceres::LinearSolverType::CGNR; // SPARSE_SCHUR // sparse normal cholesky is not threaded
	options.preconditioner_type = ceres::PreconditionerType::JACOBI;// ceres::PreconditionerType::JACOBI;// SCHUR_JACOBI;
	options.max_num_iterations = 50;// 100;
	options.logging_type = ceres::LoggingType::SILENT;
	options.minimizer_progress_to_stdout = false;
	options.function_tolerance = 1e-6;// 1e-6;
	options.num_threads = 12;
	return options;
}