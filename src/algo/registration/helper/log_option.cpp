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
	std::stringstream ss;
	ss << std::endl << "Ceres Solver:" << std::endl
		<< " Ceres preconditioner type: " << ceres_options.preconditioner_type << std::endl
		<< " Ceres linear algebra type: " << ceres_options.sparse_linear_algebra_library_type << std::endl
		<< " Ceres linear solver type: " << ceres_options.linear_solver_type << std::endl;
	logger->write(ss.str());
}

}