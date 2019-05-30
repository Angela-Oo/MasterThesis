#pragma once
//
//#include "algo/file_writer.h"
//#include "algo/surface_mesh/mesh_definition.h"
//#include "algo/mesh_simplification/mesh_simplification.h"
//#include "algo/registration/deformation_graph/deformation_graph.h"
//#include <ceres/ceres.h>
//
//typedef ml::TriMeshf Mesh;
//
//
//template <typename Registration>
//class NonRigidRegistrationAllFrames
//{
//private:
//	std::vector<SurfaceMesh> _meshes;
//	std::vector<SurfaceMesh> _deformed_meshes;
//	std::vector<DG::DeformationGraph> _deformation_graphs;
//	size_t _current;
//	unsigned int _number_of_deformation_nodes;
//	std::unique_ptr<Registration> _registration;
//public:
//	bool solve();
//	bool finished();
//	size_t getCurrent();
//	SurfaceMesh getMesh(int frame);
//	SurfaceMesh getDeformedMesh(int frame);
//	SurfaceMesh getDeformationGraphMesh(int frame);
//public:
//	NonRigidRegistrationAllFrames();
//	NonRigidRegistrationAllFrames(const std::vector<Mesh> & meshes, unsigned int number_of_deformation_nodes = 1000);
//};
//
//
//template <typename Registration>
//bool NonRigidRegistrationAllFrames<Registration>::solve()
//{
//	if (_current >= _meshes.size())
//		throw std::exception("not enouth meshes");
//	if (!_registration && _current < _meshes.size()) {
//		_registration = std::make_unique<Registration>(_meshes[0], _meshes[_current], _deformation_graphs[_current - 1], ceresOption());
//	}
//	if(_registration) {
//		if (_registration->solveIteration())
//		{
//			_deformed_meshes[_current] = _registration->getInverseDeformedPoints();
//			_deformation_graphs[_current] = _registration->getARAPDeformationGraph();
//			std::cout << std::endl << "frame " << _current << " solved" << std::endl;
//			if (_current < _meshes.size() - 1) {
//				_current++;
//				_registration.reset();
//			}
//			else {
//				return false;
//			}
//		}
//		else {
//			_deformed_meshes[_current] = _registration->getDeformedPoints();
//		}
//	}
//	return true;
//}
//
//template <typename Registration>
//bool NonRigidRegistrationAllFrames<Registration>::finished()
//{
//	return (_current >= _meshes.size() - 1);
//}
//
//template <typename Registration>
//SurfaceMesh NonRigidRegistrationAllFrames<Registration>::getMesh(int frame)
//{
//	return _meshes[frame];
//}
//
//template <typename Registration>
//SurfaceMesh NonRigidRegistrationAllFrames<Registration>::getDeformedMesh(int frame)
//{
//	return _deformed_meshes[frame];
//}
//
//template <typename Registration>
//size_t NonRigidRegistrationAllFrames<Registration>::getCurrent()
//{
//	return _current;
//}
//
//template <typename Registration>
//SurfaceMesh NonRigidRegistrationAllFrames<Registration>::getDeformationGraphMesh(int frame)
//{
//	//auto inverse_deformation = inverteDeformationGraph(_deformation_graphs[frame]);
//	//return inverse_deformation.getDeformationGraphEdges();	
//	return _deformation_graphs[frame].getDeformationGraphMesh();
//}
//
//template <typename Registration>
//NonRigidRegistrationAllFrames<Registration>::NonRigidRegistrationAllFrames()
//	: _current(1)
//{
//}
//
//template <typename Registration>
//NonRigidRegistrationAllFrames<Registration>::NonRigidRegistrationAllFrames(const std::vector<Mesh> & meshes, unsigned int number_of_deformation_nodes)
//	: _meshes(meshes)
//	, _number_of_deformation_nodes(number_of_deformation_nodes)
//	, _current(1)
//{
//	_deformation_graphs.resize(_meshes.size());
//	_deformed_meshes.resize(_meshes.size());
//
//	auto reduced_mesh = createReducedMesh(_meshes[0], _number_of_deformation_nodes);
//	_deformation_graphs[0] = DG::DeformationGraph(reduced_mesh);
//	_deformed_meshes[0] = _meshes[0];
//}
