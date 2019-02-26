#pragma once
#include "../mLibInclude.h"
#include "../kinect/DepthSensor.h"


class SensorDataWrapper
{
private:
	DepthSensor & _depth_sensor;
public:
	SensorData _data;
public:
	std::vector<vec3f> addFrame(unsigned int step = 1);
public:
	SensorDataWrapper(DepthSensor & depth_sensor,
					  mat4f color_intrinsics = mat4f::identity(),
					  mat4f depth_intrinsics = mat4f::identity());
	~SensorDataWrapper() = default;
};

class PointsFromDepthData
{
private:
	DepthSensor & _depth_sensor;
	mat4f _color_intrinsics;
	mat4f _depth_intrinsics;
public:
	std::vector<vec3f> getPoints(unsigned int step = 1);
public:
	PointsFromDepthData(DepthSensor & depth_sensor,
						mat4f color_intrinsics = mat4f::identity(),
						mat4f depth_intrinsics = mat4f::identity());
	~PointsFromDepthData() = default;
};