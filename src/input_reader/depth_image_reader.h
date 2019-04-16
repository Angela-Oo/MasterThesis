#pragma once
#include "../mLibInclude.h"
#include "i_reader.h"
#include "kinect/ImageReaderSensor.h"
#include "kinect/SensorDataWrapper.h"

class DepthImageReader : public IReader
{
private:
	std::unique_ptr<CalibrateSensorDataWrapper> _sensor_data_wrapper;
	ImageReaderSensor _depth_sensor;
private:
	void configImageReaderSensor(std::string filepath);
	ml::mat4f DepthImageReader::getWorldTransformation();
public:
	std::vector<ml::vec3f> getPoints(unsigned int frame, unsigned int step_size = 1) override;
	void processFrame() override;
	unsigned int frame() override;
public:
	void load(std::string filename) override;
	void save(std::string filename) override;
public:
	DepthImageReader(std::string filepath);
};