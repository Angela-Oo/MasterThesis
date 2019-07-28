#pragma once

#include "rigid_before_non_rigid_deformation.h"
#include "algo/registration/interface/i_registration.h"
#include "algo/registration/deformation_graph/deformation_graph.h"
#include "algo/registration/rigid_registration/rigid_registration.h"
#include "algo/surface_mesh/mesh_definition.h"


namespace Registration {

template<typename NonRigidRegistration>
class RigidBeforeNonRigidRegistration : public INonRigidRegistration
{
public:	
	using Deformation = RigidBeforeNonRigidDeformation<typename NonRigidRegistration::Deformation>;
private:
	Deformation _deformation;
	std::unique_ptr<RigidRegistration> _rigid_registration;
	std::unique_ptr<NonRigidRegistration> _non_rigid_registration;
	bool _finished_rigid_registration;
private:
	RegistrationOptions _registration_options;
public:
	bool finished() override;
	bool solveIteration() override;
	size_t currentIteration() override;
	bool solve() override;
public:
	const SurfaceMesh & getSource() override;
	const SurfaceMesh & getTarget() override;
	SurfaceMesh getDeformedPoints() override;
	SurfaceMesh getInverseDeformedPoints() override;	
public:
	SurfaceMesh getDeformationGraphMesh() override;
	const Deformation & getDeformation();
	void setRigidDeformation(const RigidDeformation & rigid_deformation) override;
	bool shouldBeSavedAsImage() override;
public:
	// without icp
	RigidBeforeNonRigidRegistration(std::unique_ptr<RigidRegistration> rigid_registration,
									std::unique_ptr<NonRigidRegistration> non_rigid_registration);
};


template<typename NonRigidRegistration>
const SurfaceMesh & RigidBeforeNonRigidRegistration<NonRigidRegistration>::getSource()
{
	return _rigid_registration->getSource();
}

template<typename NonRigidRegistration>
const SurfaceMesh & RigidBeforeNonRigidRegistration<NonRigidRegistration>::getTarget()
{
	return _rigid_registration->getTarget();
}

template<typename NonRigidRegistration>
SurfaceMesh RigidBeforeNonRigidRegistration<NonRigidRegistration>::getDeformedPoints()
{
	if (_finished_rigid_registration) {
		return _non_rigid_registration->getDeformedPoints();
	}
	else {
		return _rigid_registration->getDeformedPoints();
	}
}

template<typename NonRigidRegistration>
SurfaceMesh RigidBeforeNonRigidRegistration<NonRigidRegistration>::getInverseDeformedPoints()
{
	if (_finished_rigid_registration) {
		return _non_rigid_registration->getInverseDeformedPoints();
	}
	else {
		return _rigid_registration->getInverseDeformedPoints();
	}
}

template<typename NonRigidRegistration>
SurfaceMesh RigidBeforeNonRigidRegistration<NonRigidRegistration>::getDeformationGraphMesh()
{
	if (_finished_rigid_registration) {
		return _non_rigid_registration->getDeformationGraphMesh();
	}
	else {
		return SurfaceMesh();
	}
};

template<typename NonRigidRegistration>
bool RigidBeforeNonRigidRegistration<NonRigidRegistration>::solveIteration()
{
	if (_finished_rigid_registration) {
		return _non_rigid_registration->solveIteration();
	}
	else {
		_finished_rigid_registration = _rigid_registration->finished();
		_deformation.is_rigid_deformation = !_finished_rigid_registration;
		if (!_finished_rigid_registration) {
			_rigid_registration->solveIteration();
		}
		else {
			_non_rigid_registration->setRigidDeformation(_rigid_registration->getRigidDeformation());
		}
		return false;
	}
}

template<typename NonRigidRegistration>
size_t RigidBeforeNonRigidRegistration<NonRigidRegistration>::currentIteration()
{
	if (_finished_rigid_registration) {
		return _rigid_registration->currentIteration() + _non_rigid_registration->currentIteration();
	}
	else {
		return _rigid_registration->currentIteration();
	}
}

template<typename NonRigidRegistration>
bool RigidBeforeNonRigidRegistration<NonRigidRegistration>::solve()
{
	while (!finished()) {
		solveIteration();
	}
	return true;
}

template<typename NonRigidRegistration>
bool RigidBeforeNonRigidRegistration<NonRigidRegistration>::finished()
{
	return _non_rigid_registration->finished();
}

template<typename NonRigidRegistration>
const RigidBeforeNonRigidDeformation<typename NonRigidRegistration::Deformation> & RigidBeforeNonRigidRegistration<NonRigidRegistration>::getDeformation()
{
	_deformation.rigid_deformation = _rigid_registration->getDeformation();
	_deformation.non_rigid_deformation = _non_rigid_registration->getDeformation();
	return _deformation;
}

template<typename NonRigidRegistration>
void RigidBeforeNonRigidRegistration<NonRigidRegistration>::setRigidDeformation(const RigidDeformation & rigid_deformations)
{
	_non_rigid_registration->setRigidDeformation(rigid_deformations);
}

template<typename NonRigidRegistration>
bool RigidBeforeNonRigidRegistration<NonRigidRegistration>::shouldBeSavedAsImage()
{
	bool rigid_registration_finished = (_rigid_registration->finished() && !_finished_rigid_registration && _non_rigid_registration->currentIteration() == 0);
	bool non_rigid_finished = finished();
	return rigid_registration_finished || non_rigid_finished;
}

template<typename NonRigidRegistration>
RigidBeforeNonRigidRegistration<NonRigidRegistration>::RigidBeforeNonRigidRegistration(std::unique_ptr<RigidRegistration> rigid_registration,
																					   std::unique_ptr<NonRigidRegistration> non_rigid_registration)
	: _rigid_registration(std::move(rigid_registration))
	, _non_rigid_registration(std::move(non_rigid_registration))
	, _finished_rigid_registration(false)
{
	_deformation.rigid_deformation = _rigid_registration->getRigidDeformation();
}

}

