#include "arap_factory.h"
#include "algo/registration/helper/log_option.h"

namespace Registration {


std::unique_ptr<AsRigidAsPossible> ARAPFactory::operator()(const SurfaceMesh & source,
														   const SurfaceMesh & target)
{
	auto reduced_mesh = createReducedMesh(source, _options.dg_options.edge_length, _options.mesh_reduce_strategy);
	auto global = createGlobalDeformation(source, createDeformation);
	auto deformation_graph = createDeformationGraphFromMesh<ARAPDeformation>(reduced_mesh, global, createDeformation);

	return std::make_unique<AsRigidAsPossible>(source, target, deformation_graph, _ceres_options, _options, _logger);
}

std::unique_ptr<AsRigidAsPossible> ARAPFactory::operator()(const SurfaceMesh & source,
														   const SurfaceMesh & target,
														   const AsRigidAsPossible::Deformation & deformation_graph)
{
	return std::make_unique<AsRigidAsPossible>(source, target, deformation_graph, _ceres_options, _options, _logger);
}

void ARAPFactory::setFixedPositions(std::vector<vertex_descriptor> fixed_positions)
{
	_fixed_positions = fixed_positions;
}

SurfaceMesh ARAPFactory::deformationGraphMesh(const AsRigidAsPossible::Deformation & deformation)
{
	return deformationGraphToSurfaceMesh(deformation, _options.evaluate_residuals);
}

SurfaceMesh ARAPFactory::deformedMesh(const SurfaceMesh & mesh, const AsRigidAsPossible::Deformation & deformation)
{
	DeformedMesh<AsRigidAsPossible::Deformation> deformed(mesh, deformation, _options.dg_options.number_of_interpolation_neighbors);
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