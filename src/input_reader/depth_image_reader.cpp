#include "stdafx.h"

#include "depth_image_reader.h"



void DepthImageReader::configImageReaderSensor(std::string filepath)
{
	_depth_sensor.setBaseFilePath(filepath);
	auto frame_number = [](unsigned int idx) {
		char frameNumber_c[10];
		sprintf_s(frameNumber_c, "%06d", idx);
		return std::string(frameNumber_c);
	};
	_depth_sensor.setDepthFileName([&frame_number](unsigned int idx) { return "frame-" + frame_number(idx) + ".depth.png"; });
	_depth_sensor.setColorFileName([&frame_number](unsigned int idx) { return "frame-" + frame_number(idx) + ".color.png"; });
	_depth_sensor.setDepthIntrinsicsFileName("depthIntrinsics.txt");
	_depth_sensor.setColorIntrinsicsFileName("colorIntrinsics.txt");

	_depth_sensor.createFirstConnected();
	_depth_sensor.setNumFrames(56);
	_depth_sensor.toggleNearMode();
}

std::vector<ml::vec3f> DepthImageReader::getPoints(unsigned int frame, unsigned int step_size)
{
	return _sensor_data_wrapper->getPoints(frame, step_size);
}


void DepthImageReader::processFrame()
{
	_sensor_data_wrapper->processFrame();
}

unsigned int DepthImageReader::frame()
{
	return _sensor_data_wrapper->getNumberFrames() -1;
}

void DepthImageReader::load(std::string filename)
{
	std::cout << "load recorded frames from .sens file " << filename << std::endl;
	std::ifstream input_file;
	input_file.open(filename);
	//input_file >> _sensor_data_wrapper->_sensor_data;
	//auto frame = _sensor_data_wrapper->_sensor_data.m_frames.size() - 1;
	//auto points = _sensor_data_wrapper->getPoints(frame);
	//m_pointCloud.init(*_graphics, ml::meshutil::createPointCloudTemplate(ml::Shapesf::box(0.002f), points /*_all_points*/));
}

void DepthImageReader::save(std::string filename)
{
	std::cout << "save recorded frames as .sens file " << filename << std::endl;
	std::ofstream output_file;
	output_file.open(filename);
	//_sensor_data_wrapper->_sensor_data.saveToFile(file);
	//_sensor_data_wrapper->_sensor_data.savePointCloud(file, 0);// .saveToFile(file);
	//output_file << _sensor_data_wrapper->_sensor_data;
}


DepthImageReader::DepthImageReader(std::string filepath, ml::mat4f extrinsics)
{
	configImageReaderSensor(filepath);

	ml::mat4f depth_extrinsics = extrinsics;
	ml::mat4f color_extrinsics = extrinsics;

	_sensor_data_wrapper = std::make_unique<CalibrateSensorDataWrapper>(_depth_sensor,
																		_depth_sensor.getDepthIntrinsics(), depth_extrinsics,
																		_depth_sensor.getColorIntrinsics(), color_extrinsics);
}