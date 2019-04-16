#pragma once
#include "../mLibInclude.h"
#include "i_reader.h"
#include "kinect/PrimeSenseSensor.h"
#include "kinect/SensorDataWrapper.h"

class KinectReader : public IReader
{
private:
	std::unique_ptr<CalibrateSensorDataWrapper> _sensor_data_wrapper;
	PrimeSenseSensor _depth_sensor;
private:
	ml::mat4f getWorldTransformation();
public:
	std::vector<ml::vec3f> getPoints(unsigned int frame, unsigned int step_size = 1) override;
	void processFrame() override;
	unsigned int frame() override;
public:
	void load(std::string filename) override;
	void save(std::string filename) override;
public:
	KinectReader();
};