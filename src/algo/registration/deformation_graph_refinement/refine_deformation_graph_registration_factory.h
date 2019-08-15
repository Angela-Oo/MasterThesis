#pragma once


#include "mesh/mesh_definition.h"
#include "refine_deformation_graph_registration.h"

namespace Registration {

template<typename Factory>
class RefineDeformationGraphRegistrationFactory
{
public:
	using Registration = typename RefineDeformationGraphRegistration<typename Factory::Registration>;
	using Deformation = typename Factory::Registration::Deformation;
	using DeformMesh = typename Factory::DeformMesh;
private:
	Factory _non_rigid_factory;
public:
	std::unique_ptr<RefineDeformationGraphRegistration<typename Factory::Registration>> operator()(const SurfaceMesh & source,
																								   const SurfaceMesh & target);
	std::unique_ptr<RefineDeformationGraphRegistration<typename Factory::Registration>> operator()(const SurfaceMesh & source,
																								   const SurfaceMesh & target,
																								   const Deformation & deformation_graph);
	std::unique_ptr<RefineDeformationGraphRegistration<typename Factory::Registration>> operator()(const SurfaceMesh & source,
																								   const SurfaceMesh & target,
																								   const SurfaceMesh & previous_mesh, // used for non rigid registration
																								   const Deformation & deformation_graph);
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
	return std::make_unique<RefineDeformationGraphRegistration<typename Factory::Registration>>(_non_rigid_factory(source, target));
}

template<typename Factory>
std::unique_ptr<RefineDeformationGraphRegistration<typename Factory::Registration>>
RefineDeformationGraphRegistrationFactory<Factory>::operator()(const SurfaceMesh & source,
															   const SurfaceMesh & target,
															   const Deformation & deformation)
{
	return std::make_unique<RefineDeformationGraphRegistration<typename Factory::Registration>>(_non_rigid_factory(source, target, deformation));
}

template<typename Factory>
std::unique_ptr<RefineDeformationGraphRegistration<typename Factory::Registration>>
RefineDeformationGraphRegistrationFactory<Factory>::operator()(const SurfaceMesh & source,
															   const SurfaceMesh & target,
															   const SurfaceMesh & previous_mesh,
															   const Deformation & deformation)
{
	return std::make_unique<RefineDeformationGraphRegistration<typename Factory::Registration>>(_non_rigid_factory(source, target, previous_mesh, deformation));
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
{ }



}