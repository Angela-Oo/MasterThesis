#pragma once

#include "adaptive_rigidity.h"
#include "algo/registration/interface/i_registration.h"
#include "algo/registration/interface/registration_options.h"
#include "mesh/mesh_definition.h"
#include "util/file_writer.h"
#include <memory>


namespace Registration {

template<typename NonRigidRegistration>
class AdaptiveRigidityRegistration : public INonRigidRegistration
{
public:
	using Deformation = typename NonRigidRegistration::Deformation;
	using PositionDeformation = typename NonRigidRegistration::PositionDeformation;
	using DeformMesh = typename NonRigidRegistration::DeformMesh;
private:
	std::unique_ptr<NonRigidRegistration> _non_rigid_registration;
	bool _is_refined;
	int _number_of_refinements;
	unsigned int _current_iteration;
	bool _finished;
	RegistrationOptions _options;
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
	AdaptiveRigidityRegistration(std::unique_ptr<NonRigidRegistration> non_rigid_registration);

	AdaptiveRigidityRegistration(const SurfaceMesh& source,
								 const SurfaceMesh& target,
								 const RegistrationOptions & options,
								 std::shared_ptr<FileWriter> logger = nullptr);

	AdaptiveRigidityRegistration(const SurfaceMesh& source,
					             const SurfaceMesh& target,
					             const Deformation & deformation,
					             const RegistrationOptions & options,
					             std::shared_ptr<FileWriter> logger = nullptr);

	AdaptiveRigidityRegistration(const SurfaceMesh& source,
								 const SurfaceMesh& target,
								 const SurfaceMesh & previous_mesh,
								 const Deformation & deformation,
								 const RegistrationOptions & options,
								 std::shared_ptr<FileWriter> logger = nullptr);
};


template<typename NonRigidRegistration>
const SurfaceMesh & AdaptiveRigidityRegistration<NonRigidRegistration>::getSource()
{
	return _non_rigid_registration->getSource();
}

template<typename NonRigidRegistration>
const SurfaceMesh & AdaptiveRigidityRegistration<NonRigidRegistration>::getTarget()
{
	return _non_rigid_registration->getTarget();
}

template<typename NonRigidRegistration>
SurfaceMesh AdaptiveRigidityRegistration<NonRigidRegistration>::getDeformedPoints()
{
	return _non_rigid_registration->getDeformedPoints();
}

template<typename NonRigidRegistration>
SurfaceMesh AdaptiveRigidityRegistration<NonRigidRegistration>::getInverseDeformedPoints()
{
	return _non_rigid_registration->getInverseDeformedPoints();
}

template<typename NonRigidRegistration>
SurfaceMesh AdaptiveRigidityRegistration<NonRigidRegistration>::getDeformationGraphMesh()
{
	return _non_rigid_registration->getDeformationGraphMesh();
};

template<typename NonRigidRegistration>
bool AdaptiveRigidityRegistration<NonRigidRegistration>::solveIteration()
{	
	if (!_non_rigid_registration->getDeformation()._mesh.property_map<edge_descriptor, double>("e:rigidity").second) {
		auto deformation = _non_rigid_registration->getDeformation();
		deformation._mesh.add_property_map<edge_descriptor, double>("e:rigidity", 10.);
		_non_rigid_registration->setDeformation(deformation);
	}
	bool finished = _non_rigid_registration->finished();
	if (finished == false) {
		_current_iteration++;
		_non_rigid_registration->solveIteration();
	}
	else if(_is_refined == false) {
		auto deformation = _non_rigid_registration->getDeformation();
		auto number_adapted_edges = adaptRigidity(deformation, 
												  _options.reduce_rigidity.smooth_cost_threshold,
												  _options.reduce_rigidity.minimal_rigidity);
		_non_rigid_registration->setDeformation(deformation);
		_number_of_refinements++;
		if(number_adapted_edges == 0 || _number_of_refinements > 20)
			_is_refined = true;
	}
	else {
		_finished = true;
	}
	return _finished;
}

template<typename NonRigidRegistration>
size_t AdaptiveRigidityRegistration<NonRigidRegistration>::currentIteration()
{
	if (_current_iteration == 0)
		return _current_iteration;
	else
		return _number_of_refinements + 1;
	//return _current_iteration;
}

template<typename NonRigidRegistration>
bool AdaptiveRigidityRegistration<NonRigidRegistration>::solve()
{
	while (!finished()) {
		solveIteration();
	}
	return true;
}

template<typename NonRigidRegistration>
bool AdaptiveRigidityRegistration<NonRigidRegistration>::finished()
{
	return _finished;
}

template<typename NonRigidRegistration>
const typename NonRigidRegistration::Deformation & AdaptiveRigidityRegistration<NonRigidRegistration>::getDeformation()
{
	return _non_rigid_registration->getDeformation();
}

template<typename NonRigidRegistration>
void AdaptiveRigidityRegistration<NonRigidRegistration>::setRigidDeformation(const RigidDeformation & rigid_deformations)
{
	_non_rigid_registration->setRigidDeformation(rigid_deformations);
}

template<typename NonRigidRegistration>
bool AdaptiveRigidityRegistration<NonRigidRegistration>::shouldBeSavedAsImage()
{
	return _non_rigid_registration->shouldBeSavedAsImage();
}

template<typename NonRigidRegistration>
AdaptiveRigidityRegistration<NonRigidRegistration>::AdaptiveRigidityRegistration(std::unique_ptr<NonRigidRegistration> non_rigid_registration)
	: _non_rigid_registration(std::move(non_rigid_registration))
	, _is_refined(false)
	, _finished(false)
	, _number_of_refinements(0)
	, _current_iteration(0)
{
}

template<typename NonRigidRegistration>
AdaptiveRigidityRegistration<NonRigidRegistration>::AdaptiveRigidityRegistration(const SurfaceMesh& source,
														                         const SurfaceMesh& target,
														                         const RegistrationOptions & options,
														                         std::shared_ptr<FileWriter> logger)
	: _is_refined(false)
	, _finished(false)
	, _number_of_refinements(0)
	, _current_iteration(0)
	, _options(options)
{
	_non_rigid_registration = std::make_unique<NonRigidRegistration>(source, target, options, logger);
}

template<typename NonRigidRegistration>
AdaptiveRigidityRegistration<NonRigidRegistration>::AdaptiveRigidityRegistration(const SurfaceMesh& source,
								                                                 const SurfaceMesh& target,
								                                                 const typename NonRigidRegistration::Deformation & deformation,
								                                                 const RegistrationOptions & options,
								                                                 std::shared_ptr<FileWriter> logger)
	: _is_refined(false)
	, _finished(false)
	, _number_of_refinements(0)
	, _current_iteration(0)
	, _options(options)
{
	_non_rigid_registration = std::make_unique<NonRigidRegistration>(source, target, deformation, options, logger);
};


template<typename NonRigidRegistration>
AdaptiveRigidityRegistration<NonRigidRegistration>::AdaptiveRigidityRegistration(const SurfaceMesh& source,
																				 const SurfaceMesh& target,
																				 const SurfaceMesh& previouse_mesh,
																				 const typename NonRigidRegistration::Deformation & deformation,
																				 const RegistrationOptions & options,
																				 std::shared_ptr<FileWriter> logger)
	: _is_refined(false)
	, _finished(false)
	, _number_of_refinements(0)
	, _current_iteration(0)
	, _options(options)
{
	_non_rigid_registration = std::make_unique<NonRigidRegistration>(source, target, previouse_mesh, deformation, options, logger);
};

}

