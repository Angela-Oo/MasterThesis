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
	return _file_path + "/meshOfFrame-" + frame_number(index) + ".obj";
}

ml::TriMeshf MeshReader::getMesh(unsigned int frame)
{
	assert(frame < _meshes.size());
	return _meshes[frame];
}

void MeshReader::processFrame()
{
	std::string filename = getFileName(frame() + 1);
	ml::MeshDataf meshData = ml::MeshIOf::loadFromFile(filename);
	ml::TriMeshf triMesh(meshData);
	_meshes.push_back(triMesh);
}

unsigned int MeshReader::frame()
{
	return _meshes.size() - 1;
}

void MeshReader::load(std::string filename)
{
	std::cout << "load recorded frames from .obj file " << filename << std::endl;
}

void MeshReader::save(std::string filename)
{
	std::cout << "save recorded frames as .obj file " << filename << std::endl;
}


MeshReader::MeshReader(std::string filepath, ml::mat4f extrinsics)
	: _file_path(filepath)
{
}