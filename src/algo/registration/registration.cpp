#include "stdafx.h"

#include "registration.h"
#include "ceres_option.h"
#include "algo/registration/rigid_registration/rigid_registration.h"
#include "algo/registration/embedded_deformation/ed.h"
#include "algo/registration/arap/arap.h"
#include "algo/registration/rigid_before_non_rigid_registration/rigid_before_non_rigid_registration.h"
#include "algo/registration/helper/log_option.h"

namespace Registration {



void RegistrationFactory::setRegistrationType(RegistrationType type)
{
	_registration_type = type;
}

void RegistrationFactory::setCeresOption(const ceres::Solver::Options options)
{
	_ceres_options = options;
}

void RegistrationFactory::setRegistrationOption(const RegistrationOptions & options)
{
	_options = options;
}

void RegistrationFactory::setLogger(std::shared_ptr<FileWriter> logger)
{
	_logger = logger;
}

void RegistrationFactory::setFixedPositions(std::vector<vertex_descriptor> fixed_positions)
{
	_fixed_positions = fixed_positions;
}


std::unique_ptr<INonRigidRegistration> RegistrationFactory::buildWithoutICP(const SurfaceMesh & source,
																			const SurfaceMesh & target)
{
	if (_registration_type == RegistrationType::ED_WithoutICP) {
		if (_fixed_positions.empty())
			std::cout << "ed without icp fixed position are not set" << std::endl;
		return ED::createEmbeddedDeformation(source, target, _fixed_positions, _ceres_options, _options, _logger);
	}
	else if (_registration_type == RegistrationType::ARAP_WithoutICP) {
		if (_fixed_positions.empty())
			std::cout << "arap without icp fixed position are not set" << std::endl;
		return Registration::createAsRigidAsPossible(source, target, _fixed_positions, _ceres_options, _options, _logger);
	}
	else {
		throw("Registration type is not non rigid without icp");
		return nullptr;
	}
}

std::unique_ptr<INonRigidRegistration> RegistrationFactory::buildWithoutRigid(const SurfaceMesh & source,
																			  const SurfaceMesh & target)
{
	if (_registration_type == RegistrationType::ED_Without_RIGID) {
		return ED::createEmbeddedDeformation(source, target, _ceres_options, _options, _logger);
	}
	else if (_registration_type == RegistrationType::ARAP_Without_RIGID) {
		return Registration::createAsRigidAsPossible(source, target, _ceres_options, _options, _logger);
	}
	else {
		throw("Registration type is not non rigid without rigid");
		return nullptr;
	}
}

std::unique_ptr<INonRigidRegistration> RegistrationFactory::buildWithoutRigid(const SurfaceMesh & source,
																			  const SurfaceMesh & target,
																			  const DeformationGraph & deformation_graph)
{
	if (_registration_type == RegistrationType::ED_Without_RIGID) {
		return std::make_unique<ED::EmbeddedDeformation>(source, target, deformation_graph, _ceres_options, _options.evaluate_residuals, _logger);
	}
	else if (_registration_type == RegistrationType::ARAP_Without_RIGID) {
		return std::make_unique<AsRigidAsPossible>(source, target, deformation_graph, _ceres_options, _options, _logger);
	}
	else {
		throw("Registration type is not non rigid without rigid");
		return nullptr;
	}
}

std::unique_ptr<INonRigidRegistration> RegistrationFactory::buildWithRigid(const SurfaceMesh & source,
																		   const SurfaceMesh & target)
{
	if (_registration_type == RegistrationType::ED) {
		return std::make_unique <Registration::RigidBeforeNonRigidRegistration>(
			std::make_unique<RigidRegistration>(source, target, _ceres_options, _options, _logger),
			ED::createEmbeddedDeformation(source, target, _ceres_options, _options, _logger));
	}
	if (_registration_type == RegistrationType::ARAP) {
		return std::make_unique <Registration::RigidBeforeNonRigidRegistration>(
			std::make_unique<RigidRegistration>(source, target, _ceres_options, _options, _logger),
			createAsRigidAsPossible(source, target, _ceres_options, _options, _logger));
	}
	else {
		throw("Registration type is not non rigid");
		return nullptr;
	}
}


std::unique_ptr<INonRigidRegistration> RegistrationFactory::buildWithRigid(const SurfaceMesh & source,
																		   const SurfaceMesh & target,
																		   const DeformationGraph & deformation_graph)
{
	if (_registration_type == RegistrationType::ED) {
		return std::make_unique <Registration::RigidBeforeNonRigidRegistration>(
			std::make_unique<RigidRegistration>(source, target, deformation_graph.getRigidDeformation(), _ceres_options, _options, _logger),
			std::make_unique<ED::EmbeddedDeformation>(source, target, deformation_graph, _ceres_options, _options.evaluate_residuals, _logger));
	}
	if (_registration_type == RegistrationType::ARAP) {
		return std::make_unique <Registration::RigidBeforeNonRigidRegistration>(
			std::make_unique<RigidRegistration>(source, target, deformation_graph.getRigidDeformation(), _ceres_options, _options, _logger),
			std::make_unique<AsRigidAsPossible>(source, target, deformation_graph, _ceres_options, _options, _logger));
	}
	else {
		throw("Registration type is not non rigid");
		return nullptr;
	}
}

std::unique_ptr<INonRigidRegistration> RegistrationFactory::buildNonRigidRegistration(const SurfaceMesh & source,
																					  const SurfaceMesh & target)
{
	if (_registration_type == RegistrationType::ARAP || _registration_type == RegistrationType::ED) {
		return buildWithRigid(source, target);
	}
	else if (_registration_type == RegistrationType::ARAP_Without_RIGID || _registration_type == RegistrationType::ED_Without_RIGID) {
		return buildWithoutRigid(source, target);
	}
	else if (_registration_type == RegistrationType::ARAP_WithoutICP || _registration_type == RegistrationType::ED_WithoutICP) {
		return buildWithoutICP(source, target);
	}
	else {
		throw("Registration type is not non rigid");
		return nullptr;
	}
}

std::unique_ptr<INonRigidRegistration> RegistrationFactory::buildNonRigidRegistration(const SurfaceMesh & source,
																					  const SurfaceMesh & target,
																					  const DeformationGraph & deformation_graph)
{
	if (_registration_type == RegistrationType::ARAP || _registration_type == RegistrationType::ED) {
		return buildWithRigid(source, target, deformation_graph);
	}
	else if (_registration_type == RegistrationType::ARAP_Without_RIGID || _registration_type == RegistrationType::ED_Without_RIGID) {
		return buildWithoutRigid(source, target, deformation_graph);
	}
	else {
		throw("Registration type is not non rigid with deformation graph");
		return nullptr;
	}
}

std::unique_ptr<IRigidRegistration> RegistrationFactory::buildRigidRegistration(const SurfaceMesh & source,
																				const SurfaceMesh & target)
{
	if (_registration_type == RegistrationType::Rigid) {
		return std::make_unique<RigidRegistration>(source, target, _ceres_options, _options, _logger);
	}
	else {
		throw("Registration type is not rigid");
		return nullptr;
	}
}

std::unique_ptr<IRegistration> RegistrationFactory::buildRegistration(const SurfaceMesh & source,
																	  const SurfaceMesh & target)
{
	if (_registration_type == RegistrationType::Rigid)
		return buildRigidRegistration(source, target);
	else {
		return buildNonRigidRegistration(source, target);
	}
}

void RegistrationFactory::logConfiguration()
{
	logRegistrationOptions(_logger, _options);
	logCeresOptions(_logger, _ceres_options);
}

RegistrationFactory::RegistrationFactory() {}




}