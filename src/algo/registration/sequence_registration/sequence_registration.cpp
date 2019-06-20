#include "sequence_registration.h"
#include "algo/registration/registration.h"
#include "algo/registration/ceres_option.h"
#include "algo/registration/deformation_graph/deformed_mesh.h"
#include "algo/registration/deformation_graph/deformation_graph.h"

bool SequenceRegistration::solve()
{
	if (_current >= _meshes.size())
		throw std::exception("not enouth meshes");
	if (!_registration && _current < _meshes.size()) {
		auto & source = _meshes[0];
		auto & target = _meshes[_current];
		auto & deformation_graph = _deformation_graphs[_current - 1];
		_registration = createRegistration(source, target, _registration_type, deformation_graph, ceresOption(), _evaluate_residuals, _logger);
	}
	if (_registration) {
		auto frame_finished = _registration->finished();		
		if (frame_finished)
		{
			_deformed_meshes[_current] = _registration->getInverseDeformedPoints(); // todo
			//_deformed_meshes[_current] = _registration->getDeformedPoints();
			_deformation_graphs[_current] = _registration->getDeformationGraph();
			std::cout << std::endl << "frame " << _current << " solved" << std::endl;
			if (_current < _meshes.size() - 1) {
				_current++;
				_registration.reset();
			}
			return false;
		}
		else {
			auto finished = _registration->solveIteration();
			_deformed_meshes[_current] = _registration->getDeformedPoints();
			_deformation_graphs[_current] = _registration->getDeformationGraph();
			return finished;
		}
	}
	return true;
}

bool SequenceRegistration::finished()
{
	bool last_frame = (_current >= _meshes.size() - 1);
	if (_registration) {
		return last_frame && _registration->finished();
	}
	return last_frame;
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
	, _evaluate_residuals(false)
{
	_deformation_graphs.resize(_meshes.size());
	_deformed_meshes.resize(_meshes.size());

	auto reduced_mesh = createReducedMesh(_meshes[0], deformation_graph_edge_length);

	auto & source = _meshes[0];
	auto & target = _meshes[_current];
	_registration = createRegistration(source, target, _registration_type, ceresOption(), _evaluate_residuals, _logger, deformation_graph_edge_length);
	_deformation_graphs[0] = _registration->getDeformationGraph();
	_deformed_meshes[0] = _meshes[0];
}
