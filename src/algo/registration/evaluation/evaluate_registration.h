#pragma once

#include "algo/registration/interface/i_registration.h"
#include "algo/registration/interface/registration_options.h"
#include "mesh/mesh_definition.h"
#include "util/file_writer.h"
#include "chamfer_distance.h"
#include <memory>


namespace Registration {

template<typename NonRigidRegistration>
class EvaluateRegistration : public INonRigidRegistration
{
public:
	using Deformation = typename NonRigidRegistration::Deformation;
	using DeformMesh = typename NonRigidRegistration::DeformMesh;
private:
	std::unique_ptr<NonRigidRegistration> _non_rigid_registration;
	std::unique_ptr<CalculateChamferDistance> _calc_chamfer_distance;
	std::shared_ptr<FileWriter> _logger;
	SurfaceMesh _deformed_points;
	RegistrationOptions _options;
private:
	void errorEvaluation();
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
	EvaluateRegistration(const SurfaceMesh& source,
						 const SurfaceMesh& target,
						 const RegistrationOptions & options,
						 std::shared_ptr<FileWriter> logger = nullptr);

	EvaluateRegistration(const SurfaceMesh& source,
						 const SurfaceMesh& target,
						 const Deformation & deformation,
						 const RegistrationOptions & options,
						 std::shared_ptr<FileWriter> logger = nullptr);

	EvaluateRegistration(const SurfaceMesh& source,
						 const SurfaceMesh& target,
						 const SurfaceMesh & previous_mesh,
						 const Deformation & deformation,
						 const RegistrationOptions & options,
						 std::shared_ptr<FileWriter> logger = nullptr);
};


template <typename NonRigidRegistration>
void EvaluateRegistration<NonRigidRegistration>::errorEvaluation()
{
	_deformed_points = _non_rigid_registration->getDeformedPoints();

	auto vertex_colors = _deformed_points.property_map<vertex_descriptor, ml::vec4f>("v:color").first;

	const bool eval_error = _non_rigid_registration->finished() || _non_rigid_registration->shouldBeSavedAsImage().first;
	if (eval_error && _options.error_evaluation) {
		if (!_calc_chamfer_distance) {
			_calc_chamfer_distance = std::make_unique<CalculateChamferDistance>(_deformed_points, _non_rigid_registration->getTarget());
		}
		else {
			_calc_chamfer_distance->setSource(_deformed_points);
		}
		
		auto chamfer_distance = _calc_chamfer_distance->calculateChamferDistance();
		
		auto & source_error = chamfer_distance.source_vertices_distances;
		const double max_error = *std::max_element(source_error.second.begin(), source_error.second.end());
		for (size_t i = 0; i < source_error.first.size(); ++i) {
			vertex_colors[source_error.first[i]] = errorToRGB(source_error.second[i] / max_error, 0.9);
		}

		const ErrorStatistics error_statistic = evalErrorStatistics(chamfer_distance.distances());

		std::stringstream ss;
		ss << std::endl << "distance error: chamfer_distance " << error_statistic.sum
			<< ", mean " << error_statistic.mean << ", variance " << error_statistic.variance
			<< ", median " << error_statistic.median
			<< ", max " << error_statistic.max << ", min " << error_statistic.min;
		if (_logger)
			_logger->write(ss.str());
	}
	else {
		for (auto v : _deformed_points.vertices()) {
			vertex_colors[v] = ml::RGBColor::Cyan.toVec4f();
		}
	}
}


template<typename NonRigidRegistration>
const SurfaceMesh & EvaluateRegistration<NonRigidRegistration>::getSource()
{
	return _non_rigid_registration->getSource();
}

template<typename NonRigidRegistration>
const SurfaceMesh & EvaluateRegistration<NonRigidRegistration>::getTarget()
{
	return _non_rigid_registration->getTarget();
}

template<typename NonRigidRegistration>
SurfaceMesh EvaluateRegistration<NonRigidRegistration>::getDeformedPoints()
{
	return _deformed_points;
}

template<typename NonRigidRegistration>
SurfaceMesh EvaluateRegistration<NonRigidRegistration>::getInverseDeformedPoints()
{
	return _non_rigid_registration->getInverseDeformedPoints();
}

template<typename NonRigidRegistration>
SurfaceMesh EvaluateRegistration<NonRigidRegistration>::getDeformationGraphMesh()
{
	return _non_rigid_registration->getDeformationGraphMesh();
};

template<typename NonRigidRegistration>
bool EvaluateRegistration<NonRigidRegistration>::solveIteration()
{
	bool solved = _non_rigid_registration->solveIteration();
	errorEvaluation();
	return solved;
}

template<typename NonRigidRegistration>
double EvaluateRegistration<NonRigidRegistration>::currentError()
{
	return _non_rigid_registration->currentError();
}

template<typename NonRigidRegistration>
size_t EvaluateRegistration<NonRigidRegistration>::currentIteration()
{
	return _non_rigid_registration->currentIteration();
}

template<typename NonRigidRegistration>
bool EvaluateRegistration<NonRigidRegistration>::solve()
{
	bool solved = _non_rigid_registration->solve();
	errorEvaluation();
	return solved;
}

template<typename NonRigidRegistration>
bool EvaluateRegistration<NonRigidRegistration>::finished()
{
	return _non_rigid_registration->finished();
}

template<typename NonRigidRegistration>
const typename NonRigidRegistration::Deformation & EvaluateRegistration<NonRigidRegistration>::getDeformation()
{
	return _non_rigid_registration->getDeformation();
}

template<typename NonRigidRegistration>
void EvaluateRegistration<NonRigidRegistration>::setRigidDeformation(const RigidDeformation & rigid_deformations)
{
	_non_rigid_registration->setRigidDeformation(rigid_deformations);
}

template<typename NonRigidRegistration>
std::pair<bool, std::string> EvaluateRegistration<NonRigidRegistration>::shouldBeSavedAsImage()
{
	return _non_rigid_registration->shouldBeSavedAsImage();
}

template<typename NonRigidRegistration>
EvaluateRegistration<NonRigidRegistration>::EvaluateRegistration(const SurfaceMesh& source,
																 const SurfaceMesh& target,
																 const RegistrationOptions & options,
																 std::shared_ptr<FileWriter> logger)
	: _logger(logger)
	, _options(options)
{
	auto vertex_colors = source.property_map<vertex_descriptor, ml::vec4f>("v:color").first;
	for (auto v : source.vertices())
	{
		vertex_colors[v] = ml::RGBColor::Cyan.toVec4f();
	}
	_non_rigid_registration = std::make_unique<NonRigidRegistration>(source, target, options, logger);
	errorEvaluation();
}

template<typename NonRigidRegistration>
EvaluateRegistration<NonRigidRegistration>::EvaluateRegistration(const SurfaceMesh& source,
																 const SurfaceMesh& target,
																 const typename NonRigidRegistration::Deformation & deformation,
																 const RegistrationOptions & options,
																 std::shared_ptr<FileWriter> logger)
	: _logger(logger)
	, _options(options)
{
	auto vertex_colors = source.property_map<vertex_descriptor, ml::vec4f>("v:color").first;
	for (auto v : source.vertices())
	{
		vertex_colors[v] = ml::RGBColor::Cyan.toVec4f();
	}
	_non_rigid_registration = std::make_unique<NonRigidRegistration>(source, target, deformation, options, logger);
	errorEvaluation();
};


template<typename NonRigidRegistration>
EvaluateRegistration<NonRigidRegistration>::EvaluateRegistration(const SurfaceMesh& source,
																 const SurfaceMesh& target,
																 const SurfaceMesh& previouse_mesh,
																 const typename NonRigidRegistration::Deformation & deformation,
																 const RegistrationOptions & options,
																 std::shared_ptr<FileWriter> logger)
	: _logger(logger)
	, _options(options)
{
	auto vertex_colors = source.property_map<vertex_descriptor, ml::vec4f>("v:color").first;
	for (auto v : source.vertices())
	{
		vertex_colors[v] = ml::RGBColor::Cyan.toVec4f();
	}
	_non_rigid_registration = std::make_unique<NonRigidRegistration>(source, target, previouse_mesh, deformation, options, logger);
	errorEvaluation();
};


}