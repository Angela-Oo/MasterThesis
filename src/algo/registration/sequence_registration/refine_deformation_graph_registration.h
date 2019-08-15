#pragma once

#include "algo/registration/interface/i_registration.h"
#include "mesh/mesh_definition.h"
#include "algo/registration/deformation_graph/refine_deformation_graph.h"


namespace Registration {

template<typename NonRigidRegistration>
class RefineDeformationGraphRegistration : public INonRigidRegistration
{
public:
	using Deformation = typename NonRigidRegistration::Deformation;
private:
	Deformation _refined;
	std::unique_ptr<NonRigidRegistration> _non_rigid_registration;
	bool _is_refined;
	int _number_of_refinements;
	unsigned int _current_iteration;
	bool _finished;
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
	RefineDeformationGraphRegistration(std::unique_ptr<NonRigidRegistration> non_rigid_registration);
};


template<typename NonRigidRegistration>
const SurfaceMesh & RefineDeformationGraphRegistration<NonRigidRegistration>::getSource()
{
	return _non_rigid_registration->getSource();
}

template<typename NonRigidRegistration>
const SurfaceMesh & RefineDeformationGraphRegistration<NonRigidRegistration>::getTarget()
{
	return _non_rigid_registration->getTarget();
}

template<typename NonRigidRegistration>
SurfaceMesh RefineDeformationGraphRegistration<NonRigidRegistration>::getDeformedPoints()
{
	return _non_rigid_registration->getDeformedPoints();
}

template<typename NonRigidRegistration>
SurfaceMesh RefineDeformationGraphRegistration<NonRigidRegistration>::getInverseDeformedPoints()
{
	return _non_rigid_registration->getInverseDeformedPoints();
}

template<typename NonRigidRegistration>
SurfaceMesh RefineDeformationGraphRegistration<NonRigidRegistration>::getDeformationGraphMesh()
{
	return _non_rigid_registration->getDeformationGraphMesh();
};

template<typename NonRigidRegistration>
bool RefineDeformationGraphRegistration<NonRigidRegistration>::solveIteration()
{
	bool finished = _non_rigid_registration->finished();
	if (finished == false) {
		_current_iteration++;
		_non_rigid_registration->solveIteration();
	}
	else if(_is_refined == false) {
		_refined = refineDeformationGraph(_non_rigid_registration->getDeformation());
		_non_rigid_registration->setDeformation(_refined);
		_number_of_refinements++;
		if(_number_of_refinements > 8)
			_is_refined = true;
	}
	else {
		_finished = true;
	}
	return _finished;
}

template<typename NonRigidRegistration>
size_t RefineDeformationGraphRegistration<NonRigidRegistration>::currentIteration()
{
	//if (_current_iteration == 0)
	//	return _current_iteration;
	//else
	//	return _number_of_refinements + 1;// _current_iteration;

	return _current_iteration;
}

template<typename NonRigidRegistration>
bool RefineDeformationGraphRegistration<NonRigidRegistration>::solve()
{
	return _non_rigid_registration->solve();
}

template<typename NonRigidRegistration>
bool RefineDeformationGraphRegistration<NonRigidRegistration>::finished()
{
	return _finished;
}

template<typename NonRigidRegistration>
const typename NonRigidRegistration::Deformation & RefineDeformationGraphRegistration<NonRigidRegistration>::getDeformation()
{
	return _non_rigid_registration->getDeformation();
}

template<typename NonRigidRegistration>
void RefineDeformationGraphRegistration<NonRigidRegistration>::setRigidDeformation(const RigidDeformation & rigid_deformations)
{
	_non_rigid_registration->setRigidDeformation(rigid_deformations);
}

template<typename NonRigidRegistration>
bool RefineDeformationGraphRegistration<NonRigidRegistration>::shouldBeSavedAsImage()
{
	//bool save = _non_rigid_registration->shouldBeSavedAsImage();
	//bool registration_finished = _non_rigid_registration->finished();
	//return (save && registration_finished);
	return _non_rigid_registration->shouldBeSavedAsImage();
}

template<typename NonRigidRegistration>
RefineDeformationGraphRegistration<NonRigidRegistration>::RefineDeformationGraphRegistration(std::unique_ptr<NonRigidRegistration> non_rigid_registration)
	: _non_rigid_registration(std::move(non_rigid_registration))
	, _is_refined(false)
	, _finished(false)
	, _number_of_refinements(0)
	, _current_iteration(0)
{
}

}

