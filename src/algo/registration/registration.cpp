#include "stdafx.h"

#include "registration.h"
#include "as_rigid_as_possible.h"
#include "embedded_deformation.h"
#include "rigid_registration.h"
#include "ceres_option.h"

const Mesh & Registration::getSource()
{
	return _registration->getSource();
}

const Mesh & Registration::getTarget()
{
	return _registration->getTarget();
}

Mesh Registration::getDeformedPoints()
{
	return _registration->getDeformedPoints();
}

Mesh Registration::getInverseDeformedPoints()
{
	return _registration->getInverseDeformedPoints();
}

bool Registration::solve()
{
	return _registration->solveIteration();
}

Registration::Registration(RegistrationType _registration_type, 
						   std::shared_ptr<IMeshReader> mesh_reader, 
						   int source_frame, int target_frame,
						   std::shared_ptr<FileWriter> logger)
	: _logger(logger)
{
	auto & source = mesh_reader->getMesh(source_frame);
	auto & target = mesh_reader->getMesh(target_frame);

	auto option = ceresOption();
	if (_registration_type == RegistrationType::ED)
		_registration = std::make_unique<ED::EmbeddedDeformation>(source, target, option, _number_of_deformation_graph_nodes, _logger);
	else if (_registration_type == RegistrationType::ED_WithoutICP)
		_registration = std::make_unique<ED::EmbeddedDeformationWithoutICP>(source, target, mesh_reader->getFixedPositions(target_frame), option, _logger);
	else if (_registration_type == RegistrationType::ASAP)
		_registration = std::make_unique<AsRigidAsPossible>(source, target, option, _number_of_deformation_graph_nodes, _logger);
	else if (_registration_type == RegistrationType::ASAP_WithoutICP)
		_registration = std::make_unique<AsRigidAsPossible>(source, target, mesh_reader->getFixedPositions(target_frame), option, _logger);
	else
		_registration = std::make_unique<RigidRegistration>(source, target, option, _logger);
	
}