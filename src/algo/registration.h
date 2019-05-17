#pragma once
#include "algo/registration/icp.h"
#include "algo/registration/embedded_deformation.h"
#include "algo/registration/as_rigid_as_possible.h"
#include "algo/file_writer.h"
#include "algo/mesh_simplification/mesh_simplification.h"

typedef ml::TriMeshf Mesh;

ceres::Solver::Options ceresOption();


//class IRegistration
//{
//public:
//	virtual bool solve() = 0;
//	virtual Mesh getPointsA() = 0;
//	virtual Mesh getPointsB() = 0;
//	virtual std::vector<ml::vec3f> getPointsDeformationGraph() = 0;
//	virtual std::pair<std::vector<ml::vec3f>, std::vector<ml::vec3f>> getDeformationGraph() = 0;
//	virtual std::vector<ml::vec3f> getFixedPositions() = 0;
//	virtual ~IRegistration() = default;
//};

//class RigidRegistration : public INonRigidRegistration
//{
//private:
//	std::unique_ptr<ICPNN> _icp_nn;
//	Mesh _points_a;
//	Mesh _points_b;
//	ml::mat4f _transformation;
//	std::shared_ptr<FileWriter> _logger;
//public:
//	bool finished() override;
//	bool solveIteration() override;
//	bool solve() override;
//public:
//	const Mesh & getSource() override;
//	const Mesh & getTarget() override;
//	Mesh getDeformedPoints() override;
//	Mesh getInverseDeformedPoints() override;
//public:
//	RigidRegistration(const Mesh & points_a, const Mesh & points_b, std::shared_ptr<FileWriter> logger = nullptr);
//};


template <typename Registration, typename DeformationGraph>
class NonRigidRegistrationAllFrames
{
private:
	std::vector<Mesh> _meshes;
	std::vector<Mesh> _deformed_meshes;
	std::vector<DeformationGraph> _deformation_graphs;
	size_t _current;
	unsigned int _number_of_deformation_nodes;
	std::unique_ptr<Registration> _registration;
public:
	bool solve();
	bool finished();
	size_t getCurrent();
	Mesh getMesh(int frame);
	Mesh getDeformedMesh(int frame);
	std::pair<std::vector<ml::vec3f>, std::vector<ml::vec3f>> getDeformationGraph(int frame);
public:
	NonRigidRegistrationAllFrames();
	NonRigidRegistrationAllFrames(const std::vector<Mesh> & meshes, unsigned int number_of_deformation_nodes = 1000);
};


template <typename Registration, typename DeformationGraph>
bool NonRigidRegistrationAllFrames<Registration, DeformationGraph>::solve()
{
	if (_current >= _meshes.size())
		throw std::exception("not enouth meshes");
	if (!_registration && _current < _meshes.size()) {
		_registration = std::make_unique<Registration>(_meshes[0], _meshes[_current], _deformation_graphs[_current - 1], ceresOption(), _number_of_deformation_nodes);
		_registration->solve();
		_deformed_meshes[_current] = _registration->getInverseDeformedPoints();
		_deformation_graphs[_current] = _registration->getARAPDeformationGraph();
		std::cout << std::endl << "frame " << _current << " solved" << std::endl;
	}
	if (_current < _meshes.size() - 1) {
		
		_current++;
		_registration.reset();
		return true;
	}
	return false;
}

template <typename Registration, typename DeformationGraph>
bool NonRigidRegistrationAllFrames<Registration, DeformationGraph>::finished()
{
	return (_current >= _meshes.size() - 1);
}

template <typename Registration, typename DeformationGraph>
Mesh NonRigidRegistrationAllFrames<Registration, DeformationGraph>::getMesh(int frame)
{
	return _meshes[frame];
}

template <typename Registration, typename DeformationGraph>
Mesh NonRigidRegistrationAllFrames<Registration, DeformationGraph>::getDeformedMesh(int frame)
{
	return _deformed_meshes[frame];
}

template <typename Registration, typename DeformationGraph>
size_t NonRigidRegistrationAllFrames<Registration, DeformationGraph>::getCurrent()
{
	return _current;
}

template <typename Registration, typename DeformationGraph>
std::pair<std::vector<ml::vec3f>, std::vector<ml::vec3f>>  NonRigidRegistrationAllFrames<Registration, DeformationGraph>::getDeformationGraph(int frame)
{
	//auto inverse_deformation = inverteDeformationGraph(_deformation_graphs[frame]);
	//return inverse_deformation.getDeformationGraphEdges();	
	return _deformation_graphs[frame].getDeformationGraphEdges();
}

template <typename Registration, typename DeformationGraph>
NonRigidRegistrationAllFrames<Registration, DeformationGraph>::NonRigidRegistrationAllFrames()
	: _current(1)
{
}

template <typename Registration, typename DeformationGraph>
NonRigidRegistrationAllFrames<Registration, DeformationGraph>::NonRigidRegistrationAllFrames(const std::vector<Mesh> & meshes, unsigned int number_of_deformation_nodes)
	: _meshes(meshes)
	, _number_of_deformation_nodes(number_of_deformation_nodes)
	, _current(1)
{
	_deformation_graphs.resize(_meshes.size());
	_deformed_meshes.resize(_meshes.size());

	auto reduced_mesh = createReducedMesh(_meshes[0], _number_of_deformation_nodes);
	_deformation_graphs[0] = DeformationGraph(reduced_mesh);
	_deformed_meshes[0] = _meshes[0];
}
