#pragma once
#include "../mLibInclude.h"
#include "i_reader.h"
#include "kinect/ImageReaderSensor.h"
#include "kinect/SensorDataWrapper.h"

class MeshReader
{
private:
	std::string _file_path;
	std::string _file_name;
	std::vector<ml::TriMeshf> _meshes;
	ml::mat4f _transformation;
private:
	std::string getFileName(unsigned int index);
public:
	ml::TriMeshf& getMesh(unsigned int frame);
	bool processFrame();
	unsigned int frame();
	bool processAllFrames();
public:
	void load(std::string filename);
	void save(std::string filename);
public:
	MeshReader(std::string filepath, std::string filename, ml::mat4f transformation = ml::mat4f::identity());
};