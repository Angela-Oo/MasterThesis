#include "rigid_before_non_rigid_registration.h"

namespace Registration {

const SurfaceMesh & RigidBeforeNonRigidRegistration::getSource()
{
	return _rigid_registration->getSource();
}

const SurfaceMesh & RigidBeforeNonRigidRegistration::getTarget()
{
	return _rigid_registration->getSource();
}

SurfaceMesh RigidBeforeNonRigidRegistration::getDeformedPoints()
{
	if (_finished_rigid_registration) {
		return _non_rigid_registration->getDeformedPoints();
	}
	else {
		return _rigid_registration->getDeformedPoints();
	}
}

SurfaceMesh RigidBeforeNonRigidRegistration::getInverseDeformedPoints()
{
	if (_finished_rigid_registration) {
		return _non_rigid_registration->getInverseDeformedPoints();
	}
	else {
		return _rigid_registration->getInverseDeformedPoints();
	}
}

SurfaceMesh RigidBeforeNonRigidRegistration::getDeformationGraphMesh()
{
	if (_finished_rigid_registration) {
		return _non_rigid_registration->getDeformationGraphMesh();
	}
	else {
		return SurfaceMesh();//_rigid_registration->getDeformationGraphMesh();
	}
};


bool RigidBeforeNonRigidRegistration::solveIteration()
{
	if (_finished_rigid_registration) {
		return _non_rigid_registration->solveIteration();
	}
	else {
		bool finished = _rigid_registration->solveIteration();
		if (finished) {
			_non_rigid_registration->setRigidDeformation(_rigid_registration->getRigidDeformation());
			_finished_rigid_registration = true;
		}
		return false;
	}
}

size_t RigidBeforeNonRigidRegistration::currentIteration()
{
	if (_finished_rigid_registration) {
		return _non_rigid_registration->currentIteration();
	}
	else {
		return _rigid_registration->currentIteration();
	}
}

bool RigidBeforeNonRigidRegistration::solve()
{
	if (_finished_rigid_registration) {
		return _non_rigid_registration->solve();
	}
	else {
		_rigid_registration->solve();
		if (_rigid_registration->finished()) {
			_non_rigid_registration->setRigidDeformation(_rigid_registration->getRigidDeformation());
			_finished_rigid_registration = true;
		}
		return false;
	}
}

bool RigidBeforeNonRigidRegistration::finished()
{
	return _non_rigid_registration->finished();
}

const DG::DeformationGraph & RigidBeforeNonRigidRegistration::getDeformationGraph()
{
	return _non_rigid_registration->getDeformationGraph();
}

void RigidBeforeNonRigidRegistration::setRigidDeformation(const RigidDeformation & rigid_deformations)
{
	_non_rigid_registration->setRigidDeformation(rigid_deformations);
}


RigidBeforeNonRigidRegistration::RigidBeforeNonRigidRegistration(std::unique_ptr<IRigidRegistration> rigid_registration,
																 std::unique_ptr<INonRigidRegistration> non_rigid_registration)
	: _rigid_registration(std::move(rigid_registration))
	, _non_rigid_registration(std::move(non_rigid_registration))
	, _finished_rigid_registration(false)
{
}

}
