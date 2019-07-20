#include "sequence_registration.h"
#include "algo/registration/registration.h"
#include "algo/registration/ceres_option.h"
#include "algo/registration/deformation_graph/deformed_mesh.h"
#include "algo/registration/deformation_graph/deformation_graph.h"


void SequenceRegistration::nextFrame()
{
	_current++;
	auto & source = _meshes[0];
	auto & target = _meshes[_current];
	_deformation_graphs[_current] = _deformation_graphs[_current - 1];

	_registration = _registration_factory.buildNonRigidRegistration(source, target, _deformation_graphs[_current]);
}
bool SequenceRegistration::solve()
{
	if (_current >= _meshes.size())
		throw std::exception("not enougth meshes");
	if (!_registration && _current < _meshes.size()) {
		nextFrame();
	}
	if (_registration) {
		auto frame_finished = _registration->finished();		
		if (frame_finished)
		{
			_deformed_meshes[_current] = _registration->getDeformedPoints();
			_deformation_graphs[_current] = _registration->getDeformationGraph();
			_ceres_logger->write("frame " + std::to_string(_current) + " solved \n");
			if (_current < _meshes.size() - 1) {				
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



bool SequenceRegistration::finished()
{
	return _finished;
}

SurfaceMesh SequenceRegistration::getMesh(size_t frame)
{
	return _meshes[frame];
}

SurfaceMesh SequenceRegistration::getDeformedMesh(size_t frame)
{
	return _deformed_meshes[frame];
}

SurfaceMesh SequenceRegistration::getInverseDeformedMesh(size_t frame)
{
	auto inverse_deformation = DG::invertDeformationGraph(_deformation_graphs[frame]);
	DG::DeformedMesh deformed(_meshes[frame], inverse_deformation, _registration_options.dg_options.number_of_interpolation_neighbors);
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

SequenceRegistration::SequenceRegistration(const std::vector<SurfaceMesh> & meshes, 
										   RegistrationType registration_type, 
										   std::shared_ptr<FileWriter> logger,
										   RegistrationOptions registration_options)
	: _meshes(meshes)
	, _current(1)
	, _registration_options(registration_options)
	, _finished(false)
{
	_registration_factory.setRegistrationType(registration_type);
	_registration_factory.setCeresOption(ceresOption());
	_registration_factory.setRegistrationOption(registration_options);
	_registration_factory.setLogger(logger);

	_deformation_graphs.resize(_meshes.size());
	_deformed_meshes.resize(_meshes.size());

	auto & source = _meshes[0];
	auto & target = _meshes[_current];
	_registration = _registration_factory.buildNonRigidRegistration(source, target);

	_deformation_graphs[0] = _registration->getDeformationGraph();
	_deformed_meshes[0] = _meshes[0];
	_ceres_logger = std::make_unique<CeresLogger>(logger);
	std::string algo = (registration_type == RegistrationType::ARAP) ? "arap" : "ed";
	_ceres_logger->write("Register all frames with " + algo);

	_registration_factory.logConfiguration();
}
