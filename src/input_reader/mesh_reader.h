#pragma once
#include "../mLibInclude.h"
#include "i_reader.h"
#include "kinect/ImageReaderSensor.h"
#include "kinect/SensorDataWrapper.h"

class IMeshReader
{
public:
	virtual ml::TriMeshf& getMesh(unsigned int frame) = 0;
	virtual std::vector<int> getFixedPositions(unsigned int frame) = 0;
	virtual unsigned int frame() = 0;
};


class MeshReader : public IMeshReader
{
private:
	std::string _file_path;
	std::string _file_name;
	std::vector<ml::TriMeshf> _meshes;
	ml::mat4f _transformation;
	unsigned int _start_number;
private:
	std::string getFileName(unsigned int index);
public:
	ml::TriMeshf& getMesh(unsigned int frame) override;
	std::vector<int> getFixedPositions(unsigned int frame) override {
		return std::vector<int>();
	};
	bool processFrame();
	unsigned int frame() override;
	bool processAllFrames();
public:
	void load(std::string filename);
	void save(std::string filename);
public:
	MeshReader(std::string filepath, std::string filename, ml::mat4f transformation = ml::mat4f::identity(), unsigned int start_number = 0);
};


class DeformationMesh : public IMeshReader
{
	std::vector<ml::TriMeshf> _meshes;
public:
	ml::TriMeshf& getMesh(unsigned int frame) override;
	std::vector<int> getFixedPositions(unsigned int frame) override;
	unsigned int frame() override;
public:
	DeformationMesh();
};