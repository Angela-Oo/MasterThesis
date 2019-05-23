#include "stdafx.h"

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
	for (int i = 0; i < _cylinder_width_points; ++i) {
		fixed_index.push_back(i);
	}
	for (int i = 0; i < _cylinder_width_points; ++i) {
		fixed_index.push_back(_meshes[0].m_vertices.size() - 1 - i);
	}
	return fixed_index;
}

unsigned int DeformationMesh::frame()
{
	return _meshes.size();
}

DeformationMesh::DeformationMesh()
{
	auto mesh_1 = ml::Shapes<float>::cylinder(0.1, 1., _cylinder_height_points, _cylinder_width_points);
	_meshes.push_back(mesh_1);

	auto mesh_2 = mesh_1;

	auto rotation = ml::mat4f::rotationZ(ml::math::degreesToRadians(0.));
	auto translation = ml::mat4f::translation({ 0.4, 0., -0.4 });

	size_t num_vertices = mesh_2.m_vertices.size();
	for (int i = num_vertices - 1; i > num_vertices - _cylinder_width_points - 1; --i) {
		mesh_2.m_vertices[i].position = rotation * translation * mesh_2.m_vertices[i].position;
	}
	_meshes.push_back(mesh_2);
}




ml::TriMeshf& DeformationMeshFrames::getMesh(unsigned int frame)
{
	return _meshes[frame];
}

std::vector<int> DeformationMeshFrames::getFixedPositions(unsigned int frame)
{
	return std::vector<int>();
}

unsigned int DeformationMeshFrames::frame()
{
	return _meshes.size();
}

DeformationMeshFrames::DeformationMeshFrames()
{
	auto mesh_1 = ml::Shapes<float>::cylinder(0.1, 1., _cylinder_height_points, _cylinder_width_points);
	_meshes.push_back(mesh_1);
		
	auto rotation = ml::mat4f::rotationX(15.);
	auto translation = ml::mat4f::translation({ -0.1, 0., 0.01 });
	auto transformation = translation * rotation;

	auto mesh_2 = mesh_1;
	for (int i = 0; i < mesh_2.m_vertices.size(); ++i) {
		mesh_2.m_vertices[i].position = transformation * mesh_2.m_vertices[i].position;
	}
	_meshes.push_back(mesh_2);

	auto mesh_3 = mesh_2;
	for (int i = 0; i < mesh_3.m_vertices.size(); ++i) {
		mesh_3.m_vertices[i].position = transformation * mesh_3.m_vertices[i].position;
	}
	_meshes.push_back(mesh_3);
}
