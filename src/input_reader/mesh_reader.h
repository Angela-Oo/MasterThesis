#pragma once
#include "../mLibInclude.h"
#include "i_reader.h"
#include "kinect/ImageReaderSensor.h"
#include "kinect/SensorDataWrapper.h"

#include "algo/mesh_simplification/mesh_simplification.h"
#include "mesh/i_mesh_sequence.h"

class MeshReader : public IMeshReader
{
private:
	std::string _file_path;
	std::string _file_name;
	std::vector<SurfaceMesh> _meshes;
	ml::mat4f _transformation;
	unsigned int _start_number;
private:
	std::string getFileName(unsigned int index);
public:
	SurfaceMesh& getMesh(unsigned int frame) override;
	std::vector<vertex_descriptor> getFixedPositions(unsigned int frame) override {
		return std::vector<vertex_descriptor>();
	};
	bool processFrame();
	unsigned int frame() override;
	size_t size() override;
	bool processAllFrames();
public:
	void load(std::string filename);
	void save(std::string filename);
public:
	MeshReader(std::string filepath, std::string filename, ml::mat4f transformation = ml::mat4f::identity(), unsigned int start_number = 0);
};


class DeformationMesh : public IMeshReader
{
	std::vector<SurfaceMesh> _meshes;
	const int _cylinder_width_points = 20;
	const int _cylinder_height_points = 30;
public:
	SurfaceMesh& getMesh(unsigned int frame) override;
	std::vector<vertex_descriptor> getFixedPositions(unsigned int frame) override;
	unsigned int frame() override;
	size_t size() override;
public:
	DeformationMesh();
};

class DeformationMeshFrames : public IMeshReader
{
	std::vector<SurfaceMesh> _meshes;
	const int _cylinder_width_points = 20;
	const int _cylinder_height_points = 50;
public:
	SurfaceMesh& getMesh(unsigned int frame) override;
	std::vector<vertex_descriptor> getFixedPositions(unsigned int frame) override;
	unsigned int frame() override;
	size_t size() override;
public:
	DeformationMeshFrames();
};