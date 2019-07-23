#include "arap_factory.h"

namespace ARAP {



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


std::unique_ptr<AsRigidAsPossible> ARAPFactory::operator()(const SurfaceMesh & source,
														   const SurfaceMesh & target)
{
	auto reduced_mesh = createReducedMesh(source, _options.dg_options.edge_length, _options.mesh_reduce_strategy);
	auto global = DG::createGlobalDeformation(source, createDeformation);
	auto deformation_graph = DG::createDeformationGraphFromMesh(reduced_mesh, global, createDeformation);

	return std::make_unique<ARAP::AsRigidAsPossible>(source, target, deformation_graph, _ceres_options, _options, _logger);
}

std::unique_ptr<AsRigidAsPossible> ARAPFactory::operator()(const SurfaceMesh & source,
														   const SurfaceMesh & target,
														   const DG::DeformationGraph & deformation_graph)
{
	return std::make_unique<ARAP::AsRigidAsPossible>(source, target, deformation_graph, _ceres_options, _options, _logger);
}

void ARAPFactory::setFixedPositions(std::vector<vertex_descriptor> fixed_positions)
{
	_fixed_positions = fixed_positions;
}

SurfaceMesh ARAPFactory::deformationGraphMesh(const DG::DeformationGraph & deformation)
{
	return deformationGraphToSurfaceMesh(deformation, _options.evaluate_residuals);
}

SurfaceMesh ARAPFactory::deformedMesh(const SurfaceMesh & mesh, const DG::DeformationGraph & deformation)
{
	DG::DeformedMesh deformed(mesh, deformation, _options.dg_options.number_of_interpolation_neighbors);
	return deformed.deformPoints();
}

SurfaceMesh ARAPFactory::inverseDeformedMesh(const SurfaceMesh & mesh, const DG::DeformationGraph & deformation_graph)
{
	auto inverse_deformation = DG::invertDeformationGraph(deformation_graph);
	DG::DeformedMesh deformed(mesh, inverse_deformation, _options.dg_options.number_of_interpolation_neighbors);
	return deformed.deformPoints();
}

std::string ARAPFactory::registrationType()
{
	return "arap";
}

void ARAPFactory::logConfiguration()
{
	logRegistrationOptions(_logger, _options);
	logCeresOptions(_logger, _ceres_options);
}

ARAPFactory::ARAPFactory(const RegistrationOptions & options,
						 const ceres::Solver::Options & ceres_options,
						 std::shared_ptr<FileWriter> logger)
	: _options(options)
	, _ceres_options(ceres_options)
	, _logger(logger)
{}


}