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
		_non_rigid_registration->getDeformedPoints();
	}
	else {
		_rigid_registration->getDeformedPoints();
	}
}

SurfaceMesh RigidBeforeNonRigidRegistration::getInverseDeformedPoints()
{
	if (_finished_rigid_registration) {
		_non_rigid_registration->getInverseDeformedPoints();
	}
	else {
		_rigid_registration->getInverseDeformedPoints();
	}
}

SurfaceMesh RigidBeforeNonRigidRegistration::getDeformationGraphMesh()
{
	if (_finished_rigid_registration) {
		_non_rigid_registration->getDeformationGraphMesh();
	}
	else {
		_rigid_registration->getDeformationGraphMesh();
	}
};


bool RigidBeforeNonRigidRegistration::solveIteration()
{
	if (_finished_rigid_registration) {
		_non_rigid_registration->solveIteration();
	}
	else {
		_rigid_registration->solveIteration();
	}
}

size_t RigidBeforeNonRigidRegistration::currentIteration()
{
	if (_finished_rigid_registration) {
		_non_rigid_registration->currentIteration();
	}
	else {
		_rigid_registration->currentIteration();
	}
}

bool RigidBeforeNonRigidRegistration::solve()
{
	if (_finished_rigid_registration) {
		_non_rigid_registration->solve();
	}
	else {
		_rigid_registration->solve();
		if (_rigid_registration->finished()) {
			_non_rigid_registration = _create_non_rigid_registration(_rigid_registration->getDeformationGraph());
			_finished_rigid_registration = true;
		}
	}
}

bool RigidBeforeNonRigidRegistration::finished()
{
	if (_finished_rigid_registration) {
		_non_rigid_registration->finished();
	}
	else {
		_rigid_registration->finished();
	}
}
const DG::DeformationGraph & RigidBeforeNonRigidRegistration::getDeformationGraph()
{
	if (_finished_rigid_registration) {
		_non_rigid_registration->getDeformationGraph();
	}
	else {
		_rigid_registration->getDeformationGraph();
	}
}

RigidBeforeNonRigidRegistration::RigidBeforeNonRigidRegistration(std::unique_ptr<IRegistration> rigid_registration,
																 std::function<std::unique_ptr<IRegistration>(const DG::DeformationGraph &)> create_non_rigid_registration)
	: _rigid_registration(std::move(rigid_registration))
	, _create_non_rigid_registration(create_non_rigid_registration)
{
}

}
