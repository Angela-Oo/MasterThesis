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
												  bool evaluate_residuals,
												  std::shared_ptr<FileWriter> logger,												  
												  double deformation_graph_edge_length,
												  std::vector<vertex_descriptor> fixed_positions)
{
	if (registration_type == RegistrationType::ED)
		return std::make_unique<ED::EmbeddedDeformation>(source, target, options, deformation_graph_edge_length, evaluate_residuals, logger);
	else if (registration_type == RegistrationType::ED_WithoutICP)
		return std::make_unique<ED::EmbeddedDeformation>(source, target, fixed_positions, options, evaluate_residuals, logger);
	else if (registration_type == RegistrationType::ARAP)
		return std::make_unique<ARAP::AsRigidAsPossible>(source, target, options, deformation_graph_edge_length, evaluate_residuals, logger);
	else if (registration_type == RegistrationType::ARAP_WithoutICP)
		return std::make_unique<ARAP::AsRigidAsPossible>(source, target, fixed_positions, options, evaluate_residuals, logger);
	else
		return std::make_unique<RigidRegistration>(source, target, options, logger);
}


std::unique_ptr<IRegistration> createRegistration(const SurfaceMesh & source,
												  const SurfaceMesh & target,
												  RegistrationType registration_type,
												  DG::DeformationGraph deformation_graph,
												  const ceres::Solver::Options & options,
												  bool evaluate_residuals,
												  std::shared_ptr<FileWriter> logger)
{
	if (registration_type == RegistrationType::ED)
		return std::make_unique<ED::EmbeddedDeformation>(source, target, deformation_graph, options, evaluate_residuals, logger);
	else if (registration_type == RegistrationType::ARAP)
		return std::make_unique<ARAP::AsRigidAsPossible>(source, target, deformation_graph, options, evaluate_residuals, logger);
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
//	//else if (_registration_type == RegistrationType::ARAP)
//	//	_registration = std::make_unique<AsRigidAsPossible>(source, target, option, _number_of_deformation_graph_nodes, _logger);
//	//else if (_registration_type == RegistrationType::ARAP_WithoutICP)
//	//	_registration = std::make_unique<AsRigidAsPossible>(source, target, mesh_reader->getFixedPositions(target_frame), option, _logger);
//	//else
//	//	_registration = std::make_unique<RigidRegistration>(source, target, option, _logger);
//	
//}