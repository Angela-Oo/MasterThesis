#pragma once
#include "../mLibInclude.h"
#include "../kinect/DepthSensor.h"


class PointsFromDepthData
{
private:
	DepthSensor & _depth_sensor;
	mat4f _color_intrinsics;
	mat4f _depth_intrinsics;
public:
	PointCloudf getPoints();
	std::vector<vec3f> getPointsWithoutSensorData(); // test only
public:
	PointsFromDepthData(DepthSensor & depth_sensor,
						mat4f color_intrinsics = mat4f::identity(),
						mat4f depth_intrinsics = mat4f::identity());
	~PointsFromDepthData() = default;
};