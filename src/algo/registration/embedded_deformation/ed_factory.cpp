#include "ed_factory.h"
#include "algo/registration/util/log_option.h"
#include "algo/registration/embedded_deformation/ed.h"

namespace Registration {

using namespace ED;

std::unique_ptr<EmbeddedDeformation> EmbeddedDeformationFactory::operator()(const SurfaceMesh & source,
																			const SurfaceMesh & target)
{
	auto reduced_mesh = createReducedMesh(source, _options.dg_options.edge_length, _options.mesh_reduce_strategy);
	auto global = createGlobalDeformation<EDDeformation>(source);
	auto deformation_graph = createDeformationGraphFromMesh<EDDeformation>(reduced_mesh, global);

	return std::make_unique<EmbeddedDeformation>(source, target, deformation_graph, _ceres_options, _options, _logger);
}

std::unique_ptr<EmbeddedDeformation> EmbeddedDeformationFactory::operator()(const SurfaceMesh & source,
																			const SurfaceMesh & target,
																			const EmbeddedDeformation::Deformation & deformation_graph)
{
	return std::make_unique<EmbeddedDeformation>(source, target, deformation_graph, _ceres_options, _options, _logger);
}

std::unique_ptr<EmbeddedDeformation> EmbeddedDeformationFactory::operator()(const SurfaceMesh & source,
												                            const SurfaceMesh & target,
												                            const SurfaceMesh & previous_mesh, // used for non rigid registration
												                            const EmbeddedDeformation::Deformation & deformation_graph)
{
	return operator()(source, target, deformation_graph);
}

void EmbeddedDeformationFactory::setFixedPositions(std::vector<vertex_descriptor> fixed_positions)
{
	_fixed_positions = fixed_positions;
}

std::string EmbeddedDeformationFactory::registrationType()
{
	return "arap";
}

EmbeddedDeformationFactory::EmbeddedDeformationFactory(const RegistrationOptions & options,
						 const ceres::Solver::Options & ceres_options,
						 std::shared_ptr<FileWriter> logger)
	: _options(options)
	, _ceres_options(ceres_options)
	, _logger(logger)
{}


}