#pragma once
#include "../mLibInclude.h"
#include "../kinect/DepthSensor.h"

std::vector<ml::vec3f> getPoints(const float* depth_data,
								 unsigned int depth_width,
								 unsigned int depth_height,
								 ml::mat4f depth_intrinsics,
								 unsigned int step_size = 1);

std::vector<ml::vec3f> getPoints(ml::DepthImage32& depth_image,
								 ml::mat4f depth_intrinsics,
								 unsigned int step_size = 1);

class DepthSensorWrapper
{
public:
	DepthSensor & _depth_sensor;
public:
	// depht needs to point to array of size color width * color height
	void processDepth(unsigned short * depth);
	void processDepth(float * depth);
	// color needs to point to array of size color width * color height
	void processColor(ml::vec3uc* color);
	void processColor(ml::vec4uc* color);
	// color/depth needs to point to array of size color/depth width * color/depth height
	void processFrame(unsigned short * depth, ml::vec3uc * color);
	// color/depth needs to point to array of size color/depth width * color/depth height
	void processFrame(float * depth, ml::vec4uc * color);
public:
	unsigned int getDepthWidth();
	unsigned int getDepthHeight();
	unsigned int getColorWidth();
	unsigned int getColorHeight();
public:
	template<typename T>
	void processColorTemplate(T* color_data, std::function<T(BYTE*)> get_color)
	{
		_depth_sensor.processColor();

		auto color_rgbx = _depth_sensor.getColorRGBX();
		for (unsigned int i = 0; i < _depth_sensor.getColorHeight(); i++) {
			for (unsigned int j = 0; j < _depth_sensor.getColorWidth(); j++) {
				const unsigned int idx = (i * _depth_sensor.getColorWidth() + j) * 4;	//4 bytes per entry
				color_data[i * _depth_sensor.getColorWidth() + j] = get_color(&color_rgbx[idx]);
			}
		}
	}

	template<typename T>
	void processDepthTemplate(T* depth_data, std::function<T(float)> get_depth)
	{
		_depth_sensor.processDepth();

		for (unsigned int i = 0; i < _depth_sensor.getDepthHeight(); i++) {
			for (unsigned int j = 0; j < _depth_sensor.getDepthWidth(); j++) {
				float depth = _depth_sensor.getDepth(j, i);
				depth_data[i * _depth_sensor.getDepthWidth() + j] = get_depth(depth);
			}
		}
	}
public:
	DepthSensorWrapper(DepthSensor & depth_sensor);
	~DepthSensorWrapper() = default;
};





class PointsFromDepthData
{
private:
	DepthSensorWrapper _depth_sensor;
	ml::mat4f _color_intrinsics;
	ml::mat4f _depth_intrinsics;
	std::vector<std::vector<float>> _depth_images;
	std::vector<std::vector<ml::vec4uc>> _color_images;
public:
	void processFrame();
	std::vector<ml::vec3f> getPoints(unsigned int frame, unsigned int step_size = 1);
public:
	PointsFromDepthData(DepthSensor & depth_sensor,
						ml::mat4f color_intrinsics = ml::mat4f::identity(),
						ml::mat4f depth_intrinsics = ml::mat4f::identity());
	~PointsFromDepthData() = default;
};