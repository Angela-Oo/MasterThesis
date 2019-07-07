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
	auto & deformation_graph = _deformation_graphs[_current - 1];

	_registration = createRegistration(source, target, _registration_type, deformation_graph, ceresOption(), _evaluate_residuals, _logger);
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
			_deformed_meshes[_current] = _registration->getInverseDeformedPoints(); // todo
			//_deformed_meshes[_current] = _registration->getDeformedPoints();
			_deformation_graphs[_current] = _registration->getDeformationGraph();
			_ceres_logger->write("frame " + std::to_string(_current) + " solved");
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

size_t SequenceRegistration::getCurrent()
{
	return _current;
}

SurfaceMesh SequenceRegistration::getDeformationGraphMesh(size_t frame)
{
	//auto inverse_deformation = inverteDeformationGraph(_deformation_graphs[frame]);
	//return inverse_deformation.getDeformationGraphEdges();	

	return deformationGraphToSurfaceMesh(_deformation_graphs[frame], _evaluate_residuals);
}

SequenceRegistration::SequenceRegistration()
	: _current(1)
{
}

SequenceRegistration::SequenceRegistration(const std::vector<SurfaceMesh> & meshes, 
										   RegistrationType registration_type, 
										   std::shared_ptr<FileWriter> logger, 
										   double deformation_graph_edge_length)
	: _meshes(meshes)
	, _current(1)
	, _logger(logger)
	, _registration_type(registration_type)
	, _evaluate_residuals(true)
	, _finished(false)
{
	_deformation_graphs.resize(_meshes.size());
	_deformed_meshes.resize(_meshes.size());

	//auto reduced_mesh = createReducedMesh(_meshes[0], deformation_graph_edge_length);

	auto & source = _meshes[0];
	auto & target = _meshes[_current];
	_registration = createRegistration(source, target, _registration_type, ceresOption(), _evaluate_residuals, _logger, deformation_graph_edge_length);

	_deformation_graphs[0] = _registration->getDeformationGraph();
	_deformed_meshes[0] = _meshes[0];
	_ceres_logger = std::make_unique<CeresLogger>(_logger);
	std::string algo = (_registration_type == RegistrationType::ARAP) ? "arap" : "ed";
	_ceres_logger->write("Register all frames with " + algo);
}
