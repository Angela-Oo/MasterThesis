#pragma once

#include "mesh/i_mesh_sequence.h"
#include "algo/registration/interface/i_sequence_registration.h"
#include "util/file_writer.h"
#include "algo/registration/util/ceres_iteration_logger.h"
#include <memory>
#include <vector>

namespace Registration {

template<typename RegistrationType>
class SequenceRegistration : public ISequenceRegistration
{
public:
	using Deformation = typename RegistrationType::Deformation;
private:
	std::shared_ptr<IMeshReader> _mesh_sequence;
	std::unique_ptr<RegistrationType> _registration;
	std::vector<Deformation> _deformation;
	std::vector<SurfaceMesh> _deformed_meshes;	
	size_t _current;	
	bool _finished;
	bool _use_previouse_frame_for_rigid_registration;
	std::unique_ptr<CeresLogger> _ceres_logger;
private:
	ceres::Solver::Options _ceres_options;
	RegistrationOptions _options;
	std::shared_ptr<FileWriter> _logger;
public:
	std::pair<bool, std::string> saveCurrentFrameAsImage() override;
	SurfaceMesh getMesh(size_t frame) override;
	SurfaceMesh getDeformedMesh(size_t frame) override;
	SurfaceMesh getInverseDeformedMesh(size_t frame) override;
	SurfaceMesh getDeformationGraphMesh(size_t frame) override;
	size_t getCurrent() override;
	void nextFrame() override;
	bool finished() override;
	bool solve() override;
public:
	SequenceRegistration(std::shared_ptr<IMeshReader> mesh_sequence,
						  const RegistrationOptions & options,
						  std::shared_ptr<FileWriter> logger);
};



template<typename RegistrationType>
std::pair<bool, std::string> SequenceRegistration<RegistrationType>::saveCurrentFrameAsImage()
{
	bool save_as_image = false;
	std::string image_index = "";
	if (_registration) {
		save_as_image = _registration->shouldBeSavedAsImage();
		if (save_as_image) {
			image_index = std::to_string(getCurrent());
			if (!_registration->finished()) {
				image_index = image_index + "_arigid_" + std::to_string(_registration->currentIteration());
			}
		}
	}
	return std::make_pair(save_as_image, image_index);
}


template<typename RegistrationType>
SurfaceMesh SequenceRegistration<RegistrationType>::getMesh(size_t frame)
{
	return _mesh_sequence->getMesh(frame);
}

template<typename RegistrationType>
SurfaceMesh SequenceRegistration<RegistrationType>::getDeformedMesh(size_t frame)
{
	return _deformed_meshes[frame];
}

template<typename RegistrationType>
SurfaceMesh SequenceRegistration<RegistrationType>::getInverseDeformedMesh(size_t frame)
{
	Deformation invert_deformation = _deformation[frame].invertDeformation();
	auto deform_mesh = RegistrationType::DeformMesh(invert_deformation);
	return deform_mesh.deformedMesh(_mesh_sequence->getMesh(frame));
}

template<typename RegistrationType>
SurfaceMesh SequenceRegistration<RegistrationType>::getDeformationGraphMesh(size_t frame)
{
	auto deform_mesh = RegistrationType::DeformMesh(_deformation[frame]);
	return deform_mesh.deformationGraphMesh();
}

template<typename RegistrationType>
size_t SequenceRegistration<RegistrationType>::getCurrent()
{
	return _current;
}

template<typename RegistrationType>
void SequenceRegistration<RegistrationType>::nextFrame()
{
	_current++;
	auto & source = _mesh_sequence->getMesh(0);
	auto & target = _mesh_sequence->getMesh(_current);

	if (_use_previouse_frame_for_rigid_registration) {
		auto & prev_mesh = _mesh_sequence->getMesh(_current - 1);
		_registration = std::make_unique<RegistrationType>(source, target, prev_mesh, _deformation[_current - 1], _ceres_options, _options, _logger);
	}
	else {
		_registration = std::make_unique<RegistrationType>(source, target, _deformation[_current - 1], _ceres_options, _options, _logger);
	}
	_deformation[_current] = _registration->getDeformation();
}

template<typename RegistrationType>
bool SequenceRegistration<RegistrationType>::finished()
{
	return _finished;
}

template<typename RegistrationType>
bool SequenceRegistration<RegistrationType>::solve()
{
	if (_current >= _mesh_sequence->size())
		throw std::exception("not enougth meshes");
	if (!_registration && _current < _mesh_sequence->size()) {
		nextFrame();
		return false;
	}
	if (_registration) {
		auto frame_finished = _registration->finished();
		if (frame_finished)
		{
			_deformed_meshes[_current] = _registration->getDeformedPoints();
			_deformation[_current] = _registration->getDeformation();
			_ceres_logger->write("frame " + std::to_string(_current) + " solved \n");
			if (_current < _mesh_sequence->size() - 1) {
				_registration.reset();
			}
			else {
				_ceres_logger->write("finished registration");
				_ceres_logger.reset();
				_finished = true;
			}
			return false;
		}
		else {
			auto finished_iteration = _registration->solveIteration();
			_deformed_meshes[_current] = _registration->getDeformedPoints();
			_deformation[_current] = _registration->getDeformation();
			return finished_iteration;
		}
	}
	return true;
}

template<typename RegistrationType>
SequenceRegistration<RegistrationType>::SequenceRegistration(std::shared_ptr<IMeshReader> mesh_sequence,
														 const RegistrationOptions & options,
														 std::shared_ptr<FileWriter> logger)
	: _mesh_sequence(mesh_sequence)
	, _ceres_options(ceresOption())
	, _options(options)
	, _logger(logger)
	, _ceres_logger(std::make_unique<CeresLogger>(logger))
	, _current(1)
	, _finished(false)
	, _use_previouse_frame_for_rigid_registration(options.sequence_options.use_previouse_frame_for_rigid_registration)
{
	_deformation.resize(_mesh_sequence->size());
	_deformed_meshes.resize(_mesh_sequence->size());

	auto & source = _mesh_sequence->getMesh(0);
	auto & target = _mesh_sequence->getMesh(_current);
	_registration = std::make_unique<RegistrationType>(source, target, _ceres_options, _options, _logger);

	_deformation[0] = _registration->getDeformation();
	_deformed_meshes[0] = source;

	//std::string algo = (registration_type == RegistrationType::ARAP) ? "arap" : "ed";
	_ceres_logger->write("Register all frames with "); // TODO +_registration_factory.registrationType());
}

}