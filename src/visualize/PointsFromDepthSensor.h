#pragma once
#include "../mLibInclude.h"
#include "../kinect/DepthSensor.h"


class SensorDataWrapper
{
private:
	DepthSensor & _depth_sensor;
public:
	ml::SensorData _data;
public:
	std::vector<ml::vec3f> addFrame(unsigned int step = 1);
	std::vector<ml::vec3f> get3DPoints(unsigned int step = 1);
public:
	SensorDataWrapper(DepthSensor & depth_sensor,
					  ml::mat4f color_intrinsics = ml::mat4f::identity(),
					  ml::mat4f depth_intrinsics = ml::mat4f::identity());
	~SensorDataWrapper() = default;
};

class PointsFromDepthData
{
private:
	DepthSensor & _depth_sensor;
	ml::mat4f _color_intrinsics;
	ml::mat4f _depth_intrinsics;
public:
	std::vector<ml::vec3f> getPoints(unsigned int step = 1);
public:
	PointsFromDepthData(DepthSensor & depth_sensor,
						ml::mat4f color_intrinsics = ml::mat4f::identity(),
						ml::mat4f depth_intrinsics = ml::mat4f::identity());
	~PointsFromDepthData() = default;
};