#pragma once

#include "mesh/i_mesh_sequence.h"
#include "i_sequence_registration.h"
#include "algo/file_writer.h"
#include "algo/ceres_iteration_logger.h"
#include <memory>
#include <vector>


template<typename Registration, typename RegistrationFactory>
class SequenceRegistrationT : public ISequenceRegistration
{
public:
	typedef typename Registration::Deformation Deformation;
private:
	std::shared_ptr<IMeshReader> _mesh_sequence;
	std::unique_ptr<Registration> _registration;
	std::vector<Deformation> _deformation;
	std::vector<SurfaceMesh> _deformed_meshes;	
	size_t _current;	
	bool _finished;
	//std::function<std::unique_ptr<Registration>(const SurfaceMesh & source,
	//											const SurfaceMesh & target,
	//											Registration::Deformation deformation)> _registration_factory;
	RegistrationFactory _registration_factory;
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
	SequenceRegistrationT(std::shared_ptr<IMeshReader> mesh_sequence,
						  RegistrationFactory registration_factory,
						  std::shared_ptr<FileWriter> logger);
};



template<typename Registration, typename RegistrationFactory>
std::pair<bool, std::string> SequenceRegistrationT<Registration, RegistrationFactory>::saveCurrentFrameAsImage()
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


template<typename Registration, typename RegistrationFactory>
SurfaceMesh SequenceRegistrationT<Registration, RegistrationFactory>::getMesh(size_t frame)
{
	return _mesh_sequence->getMesh(frame);
}

template<typename Registration, typename RegistrationFactory>
SurfaceMesh SequenceRegistrationT<Registration, RegistrationFactory>::getDeformedMesh(size_t frame)
{
	return _deformed_meshes[frame];
}

template<typename Registration, typename RegistrationFactory>
SurfaceMesh SequenceRegistrationT<Registration, RegistrationFactory>::getInverseDeformedMesh(size_t frame)
{
	return _registration_factory.inverseDeformedMesh(_mesh_sequence->getMesh(frame), _deformation[frame]);
}

template<typename Registration, typename RegistrationFactory>
SurfaceMesh SequenceRegistrationT<Registration, RegistrationFactory>::getDeformationGraphMesh(size_t frame)
{
	return _registration_factory.deformationGraphMesh(_deformation[frame]);
}

template<typename Registration, typename RegistrationFactory>
size_t SequenceRegistrationT<Registration, RegistrationFactory>::getCurrent()
{
	return _current;
}

template<typename Registration, typename RegistrationFactory>
void SequenceRegistrationT<Registration, RegistrationFactory>::nextFrame()
{
	_current++;
	auto & source = _mesh_sequence->getMesh(0);
	auto & target = _mesh_sequence->getMesh(_current);

	_registration = _registration_factory(source, target, _deformation[_current - 1]);
	_deformation[_current] = _registration->getDeformationGraph();
}

template<typename Registration, typename RegistrationFactory>
bool SequenceRegistrationT<Registration, RegistrationFactory>::finished()
{
	return _finished;
}

template<typename Registration, typename RegistrationFactory>
bool SequenceRegistrationT<Registration, RegistrationFactory>::solve()
{
	if (_current >= _mesh_sequence->size())
		throw std::exception("not enougth meshes");
	if (!_registration && _current < _mesh_sequence->size()) {
		nextFrame();
	}
	if (_registration) {
		auto frame_finished = _registration->finished();
		if (frame_finished)
		{
			_deformed_meshes[_current] = _registration->getDeformedPoints();
			_deformation[_current] = _registration->getDeformationGraph();
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
			_deformation[_current] = _registration->getDeformationGraph();
			return finished_iteration;
		}
	}
	return true;
}

template<typename Registration, typename RegistrationFactory>
SequenceRegistrationT<Registration, RegistrationFactory>::SequenceRegistrationT(std::shared_ptr<IMeshReader> mesh_sequence,
																				RegistrationFactory registration_factory,
																				std::shared_ptr<FileWriter> logger)
	: _mesh_sequence(mesh_sequence)
	, _registration_factory(registration_factory)
	, _ceres_logger(std::make_unique<CeresLogger>(logger))
	, _current(1)
	, _finished(false)
{
	_deformation.resize(_mesh_sequence->size());
	_deformed_meshes.resize(_mesh_sequence->size());

	auto & source = _mesh_sequence->getMesh(0);
	auto & target = _mesh_sequence->getMesh(_current);
	_registration = _registration_factory(source, target);

	_deformation[0] = _registration->getDeformationGraph();
	_deformed_meshes[0] = source;

	//std::string algo = (registration_type == RegistrationType::ARAP) ? "arap" : "ed";
	_ceres_logger->write("Register all frames with " + _registration_factory.registrationType());

	_registration_factory.logConfiguration();
}
