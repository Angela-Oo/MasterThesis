#pragma once


#include "mesh/mesh_definition.h"
#include "refine_deformation_graph_registration.h"
#include "deformation_graph_refinement.h"
#include "refine_deformation_graph_deformation.h"
#include "refinement_deform_mesh.h"
#include "algo/registration/deformation_graph/deformation_graph.h"
#include "algo/triangulation/generate_hierarchical_mesh.h"

namespace Registration {

template<typename NonRigidRegistration>
class RefineDeformationGraphRegistrationFactory
{
public:
	using Registration = typename RefineDeformationGraphRegistration<typename NonRigidRegistration>;
private:
	ceres::Solver::Options _ceres_options;
	RegistrationOptions _options;
	std::shared_ptr<FileWriter> _logger;
public:
	std::unique_ptr<RefineDeformationGraphRegistration<typename NonRigidRegistration>> operator()(const SurfaceMesh & source,
																								   const SurfaceMesh & target);
	std::unique_ptr<RefineDeformationGraphRegistration<typename NonRigidRegistration>> operator()(const SurfaceMesh & source,
																								   const SurfaceMesh & target,
																								   const typename Registration::Deformation & deformation_graph);
	std::unique_ptr<RefineDeformationGraphRegistration<typename NonRigidRegistration>> operator()(const SurfaceMesh & source,
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




template<typename NonRigidRegistration>
std::unique_ptr<RefineDeformationGraphRegistration<typename NonRigidRegistration>>
RefineDeformationGraphRegistrationFactory<NonRigidRegistration>::operator()(const SurfaceMesh & source, const SurfaceMesh & target)
{
	auto hierarchicalMesh = generateHierarchicalMesh(source, _options.dg_options.edge_length, 4);
	auto global = createGlobalDeformation<typename Registration::PositionDeformation>(source);
	auto deformation_graph = createDeformationGraphFromMesh<typename Registration::PositionDeformation>(hierarchicalMesh._mesh, global);

	auto non_rigid_registration = std::make_unique<NonRigidRegistration>(source, target, deformation_graph, _ceres_options, _options, _logger);
	return std::make_unique<RefineDeformationGraphRegistration<typename NonRigidRegistration>>(std::move(non_rigid_registration), hierarchicalMesh);
}

template<typename NonRigidRegistration>
std::unique_ptr<RefineDeformationGraphRegistration<typename NonRigidRegistration>>
RefineDeformationGraphRegistrationFactory<NonRigidRegistration>::operator()(const SurfaceMesh & source,
															   const SurfaceMesh & target,
															   const typename RefineDeformationGraphRegistration<typename NonRigidRegistration>::Deformation & deformation)
{
	auto non_rigid_registration = std::make_unique<NonRigidRegistration>(source, target, deformation.non_rigid_deformation, _ceres_options, _options, _logger);
	return std::make_unique<RefineDeformationGraphRegistration<typename NonRigidRegistration>>(std::move(non_rigid_registration), deformation.hierarchical_mesh);
}

template<typename NonRigidRegistration>
std::unique_ptr<RefineDeformationGraphRegistration<typename NonRigidRegistration>>
RefineDeformationGraphRegistrationFactory<NonRigidRegistration>::operator()(const SurfaceMesh & source,
															   const SurfaceMesh & target,
															   const SurfaceMesh & previous_mesh,
															   const typename RefineDeformationGraphRegistration<typename NonRigidRegistration>::Deformation & deformation)
{
	return this->(source, target, deformation);
	//std::make_unique<RefineDeformationGraphRegistration<typename NonRigidRegistration>>(_non_rigid_factory(source, target, previous_mesh, deformation), deformation.hierarchical_mesh);
}

template<typename NonRigidRegistration>
void RefineDeformationGraphRegistrationFactory<NonRigidRegistration>::setFixedPositions(std::vector<vertex_descriptor> fixed_positions)
{
}

template<typename NonRigidRegistration>
std::string RefineDeformationGraphRegistrationFactory<NonRigidRegistration>::registrationType()
{
	return "todo"; // todo
	//return _non_rigid_factory.registrationType();
}

template<typename Factory>
RefineDeformationGraphRegistrationFactory<Factory>::RefineDeformationGraphRegistrationFactory(const RegistrationOptions & options,
																							  const ceres::Solver::Options & ceres_options,
																							  std::shared_ptr<FileWriter> logger)
	: _options(options)
	, _ceres_options(ceres_options)
	, _logger(logger)
{ }



}