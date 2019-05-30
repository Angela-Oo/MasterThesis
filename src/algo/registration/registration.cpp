#include "stdafx.h"

#include "registration.h"
#include "ceres_option.h"
#include "algo/registration/rigid_registration/rigid_registration.h"
#include "algo/registration/embedded_deformation/ed.h"
#include "algo/registration/arap/arap.h"

std::unique_ptr<IRegistration> createRegistration(const SurfaceMesh & source, 
												  const SurfaceMesh & target, 												  
												  RegistrationType registration_type,
												  const ceres::Solver::Options & options,
												  std::shared_ptr<FileWriter> logger,
												  int number_of_deformation_graph_nodes,
												  std::vector<vertex_descriptor> fixed_positions)
{
	if (registration_type == RegistrationType::ED)
		return std::make_unique<ED::EmbeddedDeformation>(source, target, options, number_of_deformation_graph_nodes, logger);
	else if (registration_type == RegistrationType::ED_WithoutICP)
		return std::make_unique<ED::EmbeddedDeformation>(source, target, fixed_positions, options, logger);
	else if (registration_type == RegistrationType::ASAP)
		return std::make_unique<ARAP::AsRigidAsPossible>(source, target, options, number_of_deformation_graph_nodes, logger);
	else if (registration_type == RegistrationType::ASAP_WithoutICP)
		return std::make_unique<ARAP::AsRigidAsPossible>(source, target, fixed_positions, options, logger);
	else
		return std::make_unique<RigidRegistration>(source, target, options, logger);
}


std::unique_ptr<IRegistration> createRegistration(const SurfaceMesh & source,
												  const SurfaceMesh & target,
												  RegistrationType registration_type,
												  DG::DeformationGraph deformation_graph,
												  const ceres::Solver::Options & options,
												  std::shared_ptr<FileWriter> logger)
{
	if (registration_type == RegistrationType::ED)
		return std::make_unique<ED::EmbeddedDeformation>(source, target, deformation_graph, options, logger);
	else if (registration_type == RegistrationType::ASAP)
		return std::make_unique<ARAP::AsRigidAsPossible>(source, target, deformation_graph, options, logger);
	//else if (registration_type == RegistrationType::Rigid)
	//	return std::make_unique<RigidRegistration>(source, target, options, logger);
	else
		return nullptr;
}

//
//const Mesh & Registration::getSource()
//{
//	return _registration->getSource();
//}
//
//const Mesh & Registration::getTarget()
//{
//	return _registration->getTarget();
//}
//
//Mesh Registration::getDeformedPoints()
//{
//	return _registration->getDeformedPoints();
//}
//
//Mesh Registration::getInverseDeformedPoints()
//{
//	return _registration->getInverseDeformedPoints();
//}
//
//bool Registration::solve()
//{
//	return _registration->solveIteration();
//}
//
//Registration::Registration(RegistrationType _registration_type, 
//						   std::shared_ptr<IMeshReader> mesh_reader, 
//						   int source_frame, int target_frame,
//						   std::shared_ptr<FileWriter> logger)
//	: _logger(logger)
//{
//	auto & source = mesh_reader->getMesh(source_frame);
//	auto & target = mesh_reader->getMesh(target_frame);
//
//	auto option = ceresOption();
//	//if (_registration_type == RegistrationType::ED)
//	//	_registration = std::make_unique<ED::EmbeddedDeformation>(source, target, option, _number_of_deformation_graph_nodes, _logger);
//	//else if (_registration_type == RegistrationType::ED_WithoutICP)
//	//	_registration = std::make_unique<ED::EmbeddedDeformation>(source, target, mesh_reader->getFixedPositions(target_frame), option, _logger);
//	//else if (_registration_type == RegistrationType::ASAP)
//	//	_registration = std::make_unique<AsRigidAsPossible>(source, target, option, _number_of_deformation_graph_nodes, _logger);
//	//else if (_registration_type == RegistrationType::ASAP_WithoutICP)
//	//	_registration = std::make_unique<AsRigidAsPossible>(source, target, mesh_reader->getFixedPositions(target_frame), option, _logger);
//	//else
//	//	_registration = std::make_unique<RigidRegistration>(source, target, option, _logger);
//	
//}