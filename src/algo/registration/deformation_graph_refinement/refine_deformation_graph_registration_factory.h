#pragma once


#include "mesh/mesh_definition.h"
#include "refine_deformation_graph_registration.h"
#include "deformation_graph_refinement.h"
#include "refine_deformation_graph_deformation.h"
#include "refinement_deform_mesh.h"
#include "algo/registration/deformation_graph/deformation_graph.h"
#include "algo/triangulation/generate_hierarchical_mesh.h"

namespace Registration {

template<typename Factory>
class RefineDeformationGraphRegistrationFactory
{
public:
	using Registration = typename RefineDeformationGraphRegistration<typename Factory::Registration>;
	using PositionDeformation = typename Factory::PositionDeformation;
	using DeformMesh = typename RefinementDeformMesh<typename Factory::Registration::Deformation, typename Factory::DeformMesh>;
private:
	Factory _non_rigid_factory;
	const RegistrationOptions & _options;
public:
	std::unique_ptr<RefineDeformationGraphRegistration<typename Factory::Registration>> operator()(const SurfaceMesh & source,
																								   const SurfaceMesh & target);
	std::unique_ptr<RefineDeformationGraphRegistration<typename Factory::Registration>> operator()(const SurfaceMesh & source,
																								   const SurfaceMesh & target,
																								   const typename Registration::Deformation & deformation_graph);
	std::unique_ptr<RefineDeformationGraphRegistration<typename Factory::Registration>> operator()(const SurfaceMesh & source,
																								   const SurfaceMesh & target,
																								   const SurfaceMesh & previous_mesh, // used for non rigid registration
																								   const typename Registration::Deformation & deformation_graph);
	void setFixedPositions(std::vector<vertex_descriptor> fixed_positions);
	std::string registrationType();
public:
	RefineDeformationGraphRegistrationFactory(const RegistrationOptions & options,
											  const ceres::Solver::Options & ceres_options,
											  std::shared_ptr<FileWriter> logger);
};




template<typename Factory>
std::unique_ptr<RefineDeformationGraphRegistration<typename Factory::Registration>>
RefineDeformationGraphRegistrationFactory<Factory>::operator()(const SurfaceMesh & source, const SurfaceMesh & target)
{
	auto hierarchicalMesh = generateHierarchicalMesh(source, _options.dg_options.edge_length, 4);
	auto global = createGlobalDeformation<typename Factory::PositionDeformation>(source);
	auto deformation_graph = createDeformationGraphFromMesh<typename Factory::PositionDeformation>(hierarchicalMesh._mesh, global);
	return std::make_unique<RefineDeformationGraphRegistration<typename Factory::Registration>>(_non_rigid_factory(source, target, deformation_graph), hierarchicalMesh);
}

template<typename Factory>
std::unique_ptr<RefineDeformationGraphRegistration<typename Factory::Registration>>
RefineDeformationGraphRegistrationFactory<Factory>::operator()(const SurfaceMesh & source,
															   const SurfaceMesh & target,
															   const typename RefineDeformationGraphRegistration<typename Factory::Registration>::Deformation & deformation)
{
	return std::make_unique<RefineDeformationGraphRegistration<typename Factory::Registration>>(_non_rigid_factory(source, target, deformation.non_rigid_deformation), deformation.hierarchical_mesh);
}

template<typename Factory>
std::unique_ptr<RefineDeformationGraphRegistration<typename Factory::Registration>>
RefineDeformationGraphRegistrationFactory<Factory>::operator()(const SurfaceMesh & source,
															   const SurfaceMesh & target,
															   const SurfaceMesh & previous_mesh,
															   const typename RefineDeformationGraphRegistration<typename Factory::Registration>::Deformation & deformation)
{
	return std::make_unique<RefineDeformationGraphRegistration<typename Factory::Registration>>(_non_rigid_factory(source, target, previous_mesh, deformation), deformation.hierarchical_mesh);
}

template<typename Factory>
void RefineDeformationGraphRegistrationFactory<Factory>::setFixedPositions(std::vector<vertex_descriptor> fixed_positions)
{
	_non_rigid_factory.setFixedPositions(fixed_positions);
}

template<typename Factory>
std::string RefineDeformationGraphRegistrationFactory<Factory>::registrationType()
{
	return _non_rigid_factory.registrationType();
}

template<typename Factory>
RefineDeformationGraphRegistrationFactory<Factory>::RefineDeformationGraphRegistrationFactory(const RegistrationOptions & options,
																							  const ceres::Solver::Options & ceres_options,
																							  std::shared_ptr<FileWriter> logger)
	: _non_rigid_factory(options, ceres_options, logger)
	, _options(options)
{ }



}