#include "sequence_registration.h"
#include "algo/registration/registration.h"
#include "algo/registration/ceres_option.h"
#include "algo/registration/deformation_graph/deformed_mesh.h"
#include "algo/registration/deformation_graph/deformation_graph.h"

namespace Registration {

void SequenceRegistration::nextFrame()
{
	_current++;
	auto & source = _mesh_sequence->getMesh(0);
	auto & target = _mesh_sequence->getMesh(_current);

	_registration = _registration_factory.buildNonRigidRegistration(source, target, _deformation_graphs[_current - 1]);
	_deformation_graphs[_current] = _registration->getDeformationGraph();
}

bool SequenceRegistration::solve()
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
			_deformation_graphs[_current] = _registration->getDeformationGraph();
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
			_deformation_graphs[_current] = _registration->getDeformationGraph();
			return finished_iteration;
		}
	}
	return true;
}

std::pair<bool, std::string> SequenceRegistration::saveCurrentFrameAsImage()
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

bool SequenceRegistration::finished()
{
	return _finished;
}

SurfaceMesh SequenceRegistration::getMesh(size_t frame)
{
	return _mesh_sequence->getMesh(frame);
}

SurfaceMesh SequenceRegistration::getDeformedMesh(size_t frame)
{
	return _deformed_meshes[frame];
}

SurfaceMesh SequenceRegistration::getInverseDeformedMesh(size_t frame)
{
	auto inverse_deformation = invertDeformationGraph(_deformation_graphs[frame]);
	DeformedMesh deformed(_mesh_sequence->getMesh(frame), inverse_deformation, _registration_options.dg_options.number_of_interpolation_neighbors);
	return deformed.deformPoints();
}

size_t SequenceRegistration::getCurrent()
{
	return _current;
}

SurfaceMesh SequenceRegistration::getDeformationGraphMesh(size_t frame)
{
	return deformationGraphToSurfaceMesh(_deformation_graphs[frame], _registration_options.evaluate_residuals);
}

SequenceRegistration::SequenceRegistration()
	: _current(1)
{
}

SequenceRegistration::SequenceRegistration(std::shared_ptr<IMeshReader> mesh_sequence,
										   RegistrationType registration_type,
										   std::shared_ptr<FileWriter> logger,
										   RegistrationOptions registration_options)
	: _mesh_sequence(mesh_sequence)
	, _current(1)
	, _registration_options(registration_options)
	, _finished(false)
{
	_registration_factory.setRegistrationType(registration_type);
	_registration_factory.setCeresOption(ceresOption());
	_registration_factory.setRegistrationOption(registration_options);
	_registration_factory.setLogger(logger);

	_deformation_graphs.resize(_mesh_sequence->size());
	_deformed_meshes.resize(_mesh_sequence->size());

	auto & source = _mesh_sequence->getMesh(0);
	auto & target = _mesh_sequence->getMesh(_current);
	_registration = _registration_factory.buildNonRigidRegistration(source, target);

	_deformation_graphs[0] = _registration->getDeformationGraph();
	_deformed_meshes[0] = source;
	_ceres_logger = std::make_unique<CeresLogger>(logger);
	std::string algo = (registration_type == RegistrationType::ARAP) ? "arap" : "ed";
	_ceres_logger->write("Register all frames with " + algo);

	_registration_factory.logConfiguration();
}

}