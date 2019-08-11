#include "arap_factory.h"
#include "algo/registration/util/log_option.h"


#include "algo/triangulation/generate_hierarchical_mesh.h"

namespace Registration {


std::unique_ptr<AsRigidAsPossible> ARAPFactory::operator()(const SurfaceMesh & source,
														   const SurfaceMesh & target)
{
	auto hierarchicalMesh = generateHierarchicalMesh(source, _options.dg_options.edge_length, 4);
	//auto reduced_mesh = createReducedMesh(source, _options.dg_options.edge_length, _options.mesh_reduce_strategy);
	auto global = createGlobalDeformation<ARAPDeformation>(source);
	auto deformation_graph = createDeformationGraphFromMesh<ARAPDeformation>(hierarchicalMesh, global);
	//auto deformation_graph = createDeformationGraphFromMesh<ARAPDeformation>(reduced_mesh, global);

	return std::make_unique<AsRigidAsPossible>(source, target, deformation_graph, _ceres_options, _options, _logger);
}

std::unique_ptr<AsRigidAsPossible> ARAPFactory::operator()(const SurfaceMesh & source,
														   const SurfaceMesh & target,
														   const AsRigidAsPossible::Deformation & deformation_graph)
{
	return std::make_unique<AsRigidAsPossible>(source, target, deformation_graph, _ceres_options, _options, _logger);
}

std::unique_ptr<AsRigidAsPossible> ARAPFactory::operator()(const SurfaceMesh & source,
														   const SurfaceMesh & target,
														   const SurfaceMesh & previous_mesh, // used for non rigid registration
														   const AsRigidAsPossible::Deformation & deformation_graph)
{
	return operator()(source, target, deformation_graph);
}

void ARAPFactory::setFixedPositions(std::vector<vertex_descriptor> fixed_positions)
{
	_fixed_positions = fixed_positions;
}


std::string ARAPFactory::registrationType()
{
	return "arap";
}

ARAPFactory::ARAPFactory(const RegistrationOptions & options,
						 const ceres::Solver::Options & ceres_options,
						 std::shared_ptr<FileWriter> logger)
	: _options(options)
	, _ceres_options(ceres_options)
	, _logger(logger)
{}


}