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
		_registration = createRegistration(source, target, _registration_type, deformation_graph, ceresOption(), nullptr);
	}
	if (_registration) {
		if (_registration->solveIteration())
		{
			//_deformed_meshes[_current] = _registration->getInverseDeformedPoints(); // todo
			_deformed_meshes[_current] = _registration->getDeformedPoints();
			_deformation_graphs[_current] = _registration->getDeformationGraph();
			std::cout << std::endl << "frame " << _current << " solved" << std::endl;
			if (_current < _meshes.size() - 1) {
				_current++;
				_registration.reset();
			}
			else {
				return false;
			}
		}
		else {
			_deformed_meshes[_current] = _registration->getDeformedPoints();
		}
	}
	return true;
}

bool SequenceRegistration::finished()
{
	return (_current >= _meshes.size() - 1);
}

SurfaceMesh SequenceRegistration::getMesh(int frame)
{
	return _meshes[frame];
}

SurfaceMesh SequenceRegistration::getDeformedMesh(int frame)
{
	return _deformed_meshes[frame];
}

size_t SequenceRegistration::getCurrent()
{
	return _current;
}

SurfaceMesh SequenceRegistration::getDeformationGraphMesh(int frame)
{
	//auto inverse_deformation = inverteDeformationGraph(_deformation_graphs[frame]);
	//return inverse_deformation.getDeformationGraphEdges();	

	return deformationGraphToSurfaceMesh(_deformation_graphs[frame]);
}

SequenceRegistration::SequenceRegistration()
	: _current(1)
{
}

SequenceRegistration::SequenceRegistration(const std::vector<SurfaceMesh> & meshes, RegistrationType registration_type, std::shared_ptr<FileWriter> logger, unsigned int number_of_deformation_nodes)
	: _meshes(meshes)
	, _number_of_deformation_nodes(number_of_deformation_nodes)
	, _current(1)
	, _logger(logger)
	, _registration_type(registration_type)
{
	_deformation_graphs.resize(_meshes.size());
	_deformed_meshes.resize(_meshes.size());

	auto reduced_mesh = createReducedMesh(_meshes[0], _number_of_deformation_nodes);

	auto & source = _meshes[0];
	auto & target = _meshes[_current];
	_registration = createRegistration(source, target, _registration_type, ceresOption(), logger, _number_of_deformation_nodes);
	_deformation_graphs[0] = _registration->getDeformationGraph();
	_deformed_meshes[0] = _meshes[0];
}
