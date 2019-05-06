#include "mesh_reader.h"
#include "core-mesh/meshData.h"
#include <cassert>
#include <filesystem>
#include "core-mesh/meshShapes.h"

std::string MeshReader::getFileName(unsigned int index)
{
	auto frame_number = [](unsigned int idx) {
		char frameNumber_c[10];
		sprintf_s(frameNumber_c, "%06d", idx);
		return std::string(frameNumber_c);
	};
	return _file_path + _file_name + frame_number(index) + ".obj";
}

ml::TriMeshf& MeshReader::getMesh(unsigned int frame)
{
	assert(frame < _meshes.size());
	return _meshes[frame];
}

bool MeshReader::processFrame()
{
	std::string filename = getFileName(frame() + _start_number);

	std::fstream file_stream;
	file_stream.open(filename);
	if (file_stream) {
		std::cout << "load mesh " << filename << std::endl;
		ml::MeshDataf meshData = ml::MeshIOf::loadFromFile(filename);
		ml::TriMeshf triMesh(meshData);

		triMesh.transform(_transformation);

		triMesh.computeNormals();
		triMesh.computeMeshData();
		_meshes.push_back(triMesh);
		return true;
	}
	else {
		return false;
	}
}

bool MeshReader::processAllFrames()
{
	bool successfull = true;
	while (successfull) {
		successfull = processFrame();
	}
	return true;
}

unsigned int MeshReader::frame()
{
	return _meshes.size();
}

void MeshReader::load(std::string filename)
{
	std::cout << "load recorded frames from .obj file " << filename << std::endl;
}

void MeshReader::save(std::string filename)
{
	std::cout << "save recorded frames as .obj file " << filename << std::endl;
}


MeshReader::MeshReader(std::string filepath, std::string filename, ml::mat4f transformation, unsigned int start_number)
	: _file_path(filepath)
	, _file_name(filename)
	, _transformation(transformation)
	, _start_number(start_number)
{
	auto file_name = getFileName(_start_number);
	std::fstream file_stream;
	file_stream.open(file_name);
	if (!std::experimental::filesystem::exists(_file_path)) {
		throw std::runtime_error("Directory" + _file_path + " does not exists");
	}
	else if (!file_stream) {
		throw std::runtime_error("File" + file_name + " does not exists");
	}
}





ml::TriMeshf& DeformationMesh::getMesh(unsigned int frame)
{
	return _meshes[frame];
}

std::vector<int> DeformationMesh::getFixedPositions(unsigned int frame)
{
	std::vector<int> fixed_index;
	fixed_index.push_back(0);
	fixed_index.push_back(1);
	fixed_index.push_back(2);
	fixed_index.push_back(3);
	fixed_index.push_back(4);
	fixed_index.push_back(_meshes[0].m_vertices.size() - 1);
	return fixed_index;
}

unsigned int DeformationMesh::frame()
{
	return _meshes.size();
}

DeformationMesh::DeformationMesh()
{
	auto mesh_1 = ml::Shapes<float>::cylinder(0.1, 1., 3, 5);
	_meshes.push_back(mesh_1);

	auto mesh_2 = mesh_1;
	mesh_2.m_vertices[mesh_2.m_vertices.size() - 1].position.z -= 0.4;
	mesh_2.m_vertices[mesh_2.m_vertices.size() - 1].position.x += 0.4;
	mesh_2.m_vertices[mesh_2.m_vertices.size() - 1].position.y += 0.2;
	_meshes.push_back(mesh_2);
}
