#include "ed_factory.h"
#include "algo/registration/helper/log_option.h"
#include "algo/registration/embedded_deformation/ed.h"

namespace Registration {

using namespace ED;

std::unique_ptr<EmbeddedDeformation> EmbeddedDeformationFactory::operator()(const SurfaceMesh & source,
																			const SurfaceMesh & target)
{
	auto reduced_mesh = createReducedMesh(source, _options.dg_options.edge_length, _options.mesh_reduce_strategy);
	auto global = createGlobalDeformation<ED::Deformation>(source);
	auto deformation_graph = createDeformationGraphFromMesh<ED::Deformation>(reduced_mesh, global);

	return std::make_unique<EmbeddedDeformation>(source, target, deformation_graph, _ceres_options, _options.evaluate_residuals, _logger);
}

std::unique_ptr<EmbeddedDeformation> EmbeddedDeformationFactory::operator()(const SurfaceMesh & source,
																			const SurfaceMesh & target,
																			const EmbeddedDeformation::Deformation & deformation_graph)
{
	return std::make_unique<EmbeddedDeformation>(source, target, deformation_graph, _ceres_options, _options.evaluate_residuals, _logger);
}

void EmbeddedDeformationFactory::setFixedPositions(std::vector<vertex_descriptor> fixed_positions)
{
	_fixed_positions = fixed_positions;
}

SurfaceMesh EmbeddedDeformationFactory::deformationGraphMesh(const EmbeddedDeformation::Deformation & deformation)
{
	return deformationGraphToSurfaceMesh(deformation, _options.evaluate_residuals);
}

SurfaceMesh EmbeddedDeformationFactory::deformedMesh(const SurfaceMesh & mesh, const EmbeddedDeformation::Deformation & deformation)
{
	DeformedMesh<DeformationGraph<ED::Deformation>> deformed(mesh, deformation, _options.dg_options.number_of_interpolation_neighbors);
	return deformed.deformPoints();
}


std::string EmbeddedDeformationFactory::registrationType()
{
	return "arap";
}

void EmbeddedDeformationFactory::logConfiguration()
{
	logRegistrationOptions(_logger, _options);
	logCeresOptions(_logger, _ceres_options);
}

EmbeddedDeformationFactory::EmbeddedDeformationFactory(const RegistrationOptions & options,
						 const ceres::Solver::Options & ceres_options,
						 std::shared_ptr<FileWriter> logger)
	: _options(options)
	, _ceres_options(ceres_options)
	, _logger(logger)
{}


}