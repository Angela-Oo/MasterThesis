#pragma once

#include "algo/registration/interface/i_registration.h"
#include "algo/registration/interface/registration_options.h"
#include "mesh/mesh_definition.h"
#include "util/file_writer.h"
#include "error_evaluation.h"
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
	std::unique_ptr<ErrorEvaluation> _error_evaluation;
	std::shared_ptr<FileWriter> _logger;
	SurfaceMesh _deformed_points;
	RegistrationOptions _options;
private:
	void errorEvaluation();
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

	bool eval_error = _non_rigid_registration->finished() || _non_rigid_registration->shouldBeSavedAsImage();
	if (eval_error && _options.error_evaluation) {
		if (!_error_evaluation) {
			_error_evaluation = std::make_unique<ErrorEvaluation>(_non_rigid_registration->getTarget());
		}
		auto errors = _error_evaluation->errorEvaluation(_deformed_points);
		
		double max_error = errors.max();
		for (size_t i = 0; i < errors.size(); ++i)
		{
			vertex_colors[errors.v(i)] = errorToRGB(errors.error(i) / max_error, 0.9);
		}

		std::stringstream ss;
		ss << std::endl << "error: mean " << errors.mean() << ", variance " << errors.variance() << ", median " << errors.median() << ", max " << errors.max();
		if (_logger)
			_logger->write(ss.str());
	}
	else
	{
		for (auto v : _deformed_points.vertices())
		{
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
bool EvaluateRegistration<NonRigidRegistration>::shouldBeSavedAsImage()
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