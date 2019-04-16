#pragma once
#include "../mLibInclude.h"
#include "../kinect/DepthSensor.h"
#include "../kinect/DepthSensorWrapper.h"


class SensorDataWrapper
{
private:
	DepthSensorWrapper _depth_sensor;
public:
	ml::SensorData _sensor_data;
public:
	void processFrame();
	std::vector<ml::vec3f> getPoints(unsigned int frame, unsigned int step_size = 1);
public:
	SensorDataWrapper(DepthSensor & depth_sensor,
					  ml::mat4f color_intrinsics = ml::mat4f::identity(),
					  ml::mat4f depth_intrinsics = ml::mat4f::identity());
	~SensorDataWrapper() = default;
};


class CalibrateSensorDataWrapper
{
private:
	DepthSensorWrapper _depth_sensor;
public:
	ml::CalibratedSensorData _sensor_data;
public:
	void processFrame();
	unsigned int getNumberFrames();
	std::vector<ml::vec3f> getPoints(unsigned int frame, unsigned int step_size = 1);
	ml::SensorData getSensorData();
public:
	CalibrateSensorDataWrapper(DepthSensor & depth_sensor, 
							   ml::mat4f depth_intrinsics, ml::mat4f depth_extrinsics,
							   ml::mat4f color_intrinsics, ml::mat4f color_extrinsics);
	~CalibrateSensorDataWrapper();
};

