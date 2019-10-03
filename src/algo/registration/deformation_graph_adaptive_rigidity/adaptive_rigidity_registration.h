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
	double _last_cost{ 2. };
	bool _is_refined;
	int _number_of_refinements;
	unsigned int _current_iteration;
	bool _finished;
	RegistrationOptions _options;
	std::shared_ptr<FileWriter> _logger;
	bool _need_refinement {false};
private:
	bool needRefinement();
public:
	bool finished() override;
	bool solveIteration() override;
	double currentError() override;
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
	std::pair<bool, std::string> shouldBeSavedAsImage() override;
public:
	//AdaptiveRigidityRegistration(std::unique_ptr<NonRigidRegistration> non_rigid_registration);
	void initEdgeRigidity();

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
}

template<typename NonRigidRegistration>
bool AdaptiveRigidityRegistration<NonRigidRegistration>::needRefinement()
{
	auto scale_factor_tol = 0.1;
	auto current_cost = _non_rigid_registration->currentError();
	if (abs(current_cost - _last_cost) < (scale_factor_tol * current_cost))
	{
		return true;
	}
	return false;
}

template<typename NonRigidRegistration>
bool AdaptiveRigidityRegistration<NonRigidRegistration>::solveIteration()
{
	if (!_need_refinement) {
		_current_iteration++;
		_last_cost = _non_rigid_registration->currentError();
		bool solved = _non_rigid_registration->solveIteration();
		if (solved || _current_iteration >= _options.max_iterations) {
			_finished = true;
		}
		else {
			_need_refinement = needRefinement();
		}
	}
	else if(_is_refined == false) {
		_need_refinement = false;
		auto &deformation = _non_rigid_registration->getDeformationMesh();
		double minimal_rigidity = _options.reduce_rigidity.minimal_rigidity / _options.smooth;// normalize with the smooth factor to make the value absolut TODO check for side effects
		auto number_adapted_edges = adaptRigidity(deformation,
												  _options.reduce_rigidity.rigidity_cost_threshold,
												  minimal_rigidity);// _options.reduce_rigidity.minimal_rigidity);

		if (number_adapted_edges > 0) {			
			_number_of_refinements++;
			if (_logger) {
				_logger->write("\n reduced rigidity for " + std::to_string(number_adapted_edges) + " edges ");
			}
			unsigned int max_number_of_refinement = (_options.sequence_options.enable) ? 5 : 20;
			if (_number_of_refinements >= max_number_of_refinement)
				_is_refined = true;
		}
		else {
			_is_refined = true;
		}
	}
	else {
		_finished = true;
	}
	return _finished;
}

template<typename NonRigidRegistration>
double AdaptiveRigidityRegistration<NonRigidRegistration>::currentError()
{
	return _non_rigid_registration->currentError();
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
	//initEdgeRigidity();
}

template<typename NonRigidRegistration>
std::pair<bool, std::string> AdaptiveRigidityRegistration<NonRigidRegistration>::shouldBeSavedAsImage()
{
	auto save_image = _non_rigid_registration->shouldBeSavedAsImage();
	if (save_image.first) {
		save_image.second = "reduce_" + std::to_string(_number_of_refinements) + "_" + save_image.second;
	}
	else if (_finished)
		return std::make_pair(true, "reduce_" + std::to_string(_number_of_refinements));
	return save_image;
}
//
//template<typename NonRigidRegistration>
//AdaptiveRigidityRegistration<NonRigidRegistration>::AdaptiveRigidityRegistration(std::unique_ptr<NonRigidRegistration> non_rigid_registration)
//	: _non_rigid_registration(std::move(non_rigid_registration))
//	, _is_refined(false)
//	, _finished(false)
//	, _number_of_refinements(0)
//	, _current_iteration(0)
//{
//}

template <typename NonRigidRegistration>
void AdaptiveRigidityRegistration<NonRigidRegistration>::initEdgeRigidity()
{
	auto & deformation = _non_rigid_registration->getDeformationMesh();
	if (!deformation.property_map<edge_descriptor, double>("e:rigidity").second) {
		deformation.add_property_map<edge_descriptor, double>("e:rigidity", 1.);
	}
	else
	{
		auto & rigidity = deformation.property_map<edge_descriptor, double>("e:rigidity").first;
		for (auto e : deformation.edges())
			rigidity[e] = 1.;
	}
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
	, _logger(logger)
{
	_non_rigid_registration = std::make_unique<NonRigidRegistration>(source, target, options, logger);
	initEdgeRigidity();
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
	, _logger(logger)
{
	_non_rigid_registration = std::make_unique<NonRigidRegistration>(source, target, deformation, options, logger);
	initEdgeRigidity();
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
	, _logger(logger)
{
	_non_rigid_registration = std::make_unique<NonRigidRegistration>(source, target, previouse_mesh, deformation, options, logger);
	initEdgeRigidity();
};

}

