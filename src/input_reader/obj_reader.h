#pragma once
#include "../mLibInclude.h"
#include "i_reader.h"
#include "kinect/ImageReaderSensor.h"
#include "kinect/SensorDataWrapper.h"

class MeshReader
{
private:
	std::string _file_path;
	std::vector<ml::TriMeshf> _meshes;
private:
	std::string getFileName(unsigned int index);
public:
	ml::TriMeshf getMesh(unsigned int frame);
	void processFrame();
	unsigned int frame();
public:
	void load(std::string filename);
	void save(std::string filename);
public:
	MeshReader(std::string filepath, ml::mat4f extrinsics = ml::mat4f::identity());
};