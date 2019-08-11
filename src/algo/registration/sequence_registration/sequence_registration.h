#pragma once

#include "mesh/i_mesh_sequence.h"
#include "algo/registration/interface/i_sequence_registration.h"
#include "util/file_writer.h"
#include "algo/registration/util/ceres_iteration_logger.h"
#include <memory>
#include <vector>

namespace Registration {

template<typename RegistrationFactory>
class SequenceRegistration : public ISequenceRegistration
{
public:
	using Registration = typename RegistrationFactory::Registration;
	using Deformation = typename RegistrationFactory::Registration::Deformation;
private:
	std::shared_ptr<IMeshReader> _mesh_sequence;
	std::unique_ptr<Registration> _registration;
	std::vector<Deformation> _deformation;
	std::vector<SurfaceMesh> _deformed_meshes;	
	size_t _current;	
	bool _finished;
	RegistrationFactory _registration_factory;
	bool _use_previouse_frame_for_rigid_registration;
	std::unique_ptr<CeresLogger> _ceres_logger;
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
						  RegistrationFactory registration_factory,
						  const RegistrationOptions & options,
						  std::shared_ptr<FileWriter> logger);
};



template<typename RegistrationFactory>
std::pair<bool, std::string> SequenceRegistration<RegistrationFactory>::saveCurrentFrameAsImage()
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


template<typename RegistrationFactory>
SurfaceMesh SequenceRegistration<RegistrationFactory>::getMesh(size_t frame)
{
	return _mesh_sequence->getMesh(frame);
}

template<typename RegistrationFactory>
SurfaceMesh SequenceRegistration<RegistrationFactory>::getDeformedMesh(size_t frame)
{
	return _deformed_meshes[frame];
}

template<typename RegistrationFactory>
SurfaceMesh SequenceRegistration<RegistrationFactory>::getInverseDeformedMesh(size_t frame)
{
	Deformation invert_deformation = _deformation[frame].invertDeformation();
	auto deform_mesh = RegistrationFactory::DeformMesh(invert_deformation);
	return deform_mesh.deformedMesh(_mesh_sequence->getMesh(frame));
}

template<typename RegistrationFactory>
SurfaceMesh SequenceRegistration<RegistrationFactory>::getDeformationGraphMesh(size_t frame)
{
	auto deform_mesh = RegistrationFactory::DeformMesh(_deformation[frame]);
	return deform_mesh.deformationGraphMesh();
}

template<typename RegistrationFactory>
size_t SequenceRegistration<RegistrationFactory>::getCurrent()
{
	return _current;
}

template<typename RegistrationFactory>
void SequenceRegistration<RegistrationFactory>::nextFrame()
{
	_current++;
	auto & source = _mesh_sequence->getMesh(0);
	auto & target = _mesh_sequence->getMesh(_current);

	if (_use_previouse_frame_for_rigid_registration) {
		auto & prev_mesh = _mesh_sequence->getMesh(_current - 1);
		_registration = _registration_factory(source, target, prev_mesh, _deformation[_current - 1]);
	}
	else {
		_registration = _registration_factory(source, target, _deformation[_current - 1]);
	}
	_deformation[_current] = _registration->getDeformation();
}

template<typename RegistrationFactory>
bool SequenceRegistration<RegistrationFactory>::finished()
{
	return _finished;
}

template<typename RegistrationFactory>
bool SequenceRegistration<RegistrationFactory>::solve()
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

template<typename RegistrationFactory>
SequenceRegistration<RegistrationFactory>::SequenceRegistration(std::shared_ptr<IMeshReader> mesh_sequence,
																				RegistrationFactory registration_factory,
																				const RegistrationOptions & options,
																				std::shared_ptr<FileWriter> logger)
	: _mesh_sequence(mesh_sequence)
	, _registration_factory(registration_factory)
	, _ceres_logger(std::make_unique<CeresLogger>(logger))
	, _current(1)
	, _finished(false)
	, _use_previouse_frame_for_rigid_registration(options.sequence_options.use_previouse_frame_for_rigid_registration)
{
	_deformation.resize(_mesh_sequence->size());
	_deformed_meshes.resize(_mesh_sequence->size());

	auto & source = _mesh_sequence->getMesh(0);
	auto & target = _mesh_sequence->getMesh(_current);
	_registration = _registration_factory(source, target);

	_deformation[0] = _registration->getDeformation();
	_deformed_meshes[0] = source;

	//std::string algo = (registration_type == RegistrationType::ARAP) ? "arap" : "ed";
	_ceres_logger->write("Register all frames with " + _registration_factory.registrationType());
}

}