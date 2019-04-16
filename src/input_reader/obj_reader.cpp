#include "obj_reader.h"
#include "core-mesh/meshData.h"
#include <cassert>

std::string MeshReader::getFileName(unsigned int index)
{
	auto frame_number = [](unsigned int idx) {
		char frameNumber_c[10];
		sprintf_s(frameNumber_c, "%06d", idx);
		return std::string(frameNumber_c);
	};
	return _file_path + "meshOfFrame" + frame_number(index) + ".obj";
}

ml::TriMeshf& MeshReader::getMesh(unsigned int frame)
{
	assert(frame <= _meshes.size());
	return _meshes[frame - 1];
}

void MeshReader::processFrame()
{
	std::string filename = getFileName(frame() + 1);
	try {
		ml::MeshDataf meshData = ml::MeshIOf::loadFromFile(filename);
		ml::TriMeshf triMesh(meshData);

		auto bounding_box = triMesh.computeBoundingBox();
		ml::mat4f center = ml::mat4f::translation(-bounding_box.getCenter());
		triMesh.transform(center);

		triMesh.transform(_transformation);

		triMesh.computeNormals();
		triMesh.computeMeshData();
		_meshes.push_back(triMesh);
	}
	catch (...) {
		std::cout << "could not load file " << filename << std::endl;
	}
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


MeshReader::MeshReader(std::string filepath, ml::mat4f transformation)
	: _file_path(filepath)
	, _transformation(transformation)
{
}