#include "DepthSensorWrapper.h"

using namespace ml;

std::vector<ml::vec3f> getPoints(const float* depth_data,
								 unsigned int depth_width,
								 unsigned int depth_height,
								 ml::mat4f depth_intrinsics,
								 unsigned int step_size)
{
	std::vector<ml::vec3f> points;

	ml::mat4f depth_intrinsics_inv = depth_intrinsics;
	depth_intrinsics_inv.invert();

	for (unsigned int y = 0; y < depth_height; y += step_size) {
		for (unsigned int x = 0; x < depth_width; x += step_size) {
			float depth = depth_data[x + y * depth_width];
			vec3f p(static_cast<float>(x), static_cast<float>(y), 1.);
			p = depth_intrinsics_inv * p;
			p = p * depth;
			points.push_back(p);
		}
	}
	return points;
}


std::vector<ml::vec3f> getPoints(ml::DepthImage32& depth_image,
								 ml::mat4f depth_intrinsics,
								 unsigned int step_size)
{
	float * depth_data = depth_image.getData();
	return ::getPoints(depth_data, depth_image.getWidth(), depth_image.getHeight(), depth_intrinsics, step_size);
}






void DepthSensorWrapper::processDepth(unsigned short * depth_data)
{
	auto get_depth = [](float depth) { return static_cast<unsigned short>(depth * 1000.); };
	processDepthTemplate<unsigned short>(depth_data, get_depth);
}

void DepthSensorWrapper::processDepth(float * depth_data)
{
	auto get_depth = [](float depth) { return depth; };
	processDepthTemplate<float>(depth_data, get_depth);
}

void DepthSensorWrapper::processColor(ml::vec3uc * color_data)
{
	auto get_color = [](BYTE * color) { return ml::vec3uc(color[0], color[1], color[2]); };
	processColorTemplate<ml::vec3uc>(color_data, get_color);
}

void DepthSensorWrapper::processColor(ml::vec4uc * color_data)
{
	auto get_color = [](BYTE * color) { return ml::vec4uc(color[0], color[1], color[2], 0); };
	processColorTemplate<ml::vec4uc>(color_data, get_color);
}

void DepthSensorWrapper::processFrame(unsigned short * depth, ml::vec3uc * color)
{
	processDepth(depth);
	processColor(color);
}

void DepthSensorWrapper::processFrame(float * depth, ml::vec4uc * color)
{
	processDepth(depth);
	processColor(color);
}

unsigned int DepthSensorWrapper::getDepthWidth() {
	return _depth_sensor.getDepthWidth();
}

unsigned int DepthSensorWrapper::getDepthHeight(){
	return _depth_sensor.getDepthHeight();
}

unsigned int DepthSensorWrapper::getColorWidth() {
	return _depth_sensor.getColorWidth();
}

unsigned int DepthSensorWrapper::getColorHeight() {
	return _depth_sensor.getColorHeight();
}

DepthSensorWrapper::DepthSensorWrapper(DepthSensor & depth_sensor)
	: _depth_sensor(depth_sensor)
{
}



//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------



void PointsFromDepthData::processFrame()
{
	_depth_images.emplace_back(std::vector<float>(_depth_sensor._depth_sensor.getDepthWidth() * _depth_sensor._depth_sensor.getDepthHeight()));
	_color_images.emplace_back(std::vector<ml::vec4uc>(_depth_sensor._depth_sensor.getColorHeight() * _depth_sensor._depth_sensor.getColorWidth()));

	_depth_sensor.processFrame(_depth_images.back().data(), _color_images.back().data());
	
}

std::vector<ml::vec3f> PointsFromDepthData::getPoints(unsigned int frame, unsigned int step_size)
{
	std::vector<ml::vec3f> points;
	if (frame >= _depth_images.size())
		return points;
	
	//return ::getPoints(_depth_images[frame].data(), _depth_sensor._depth_sensor.getDepthWidth(), _depth_sensor._depth_sensor.getDepthHeight(), _depth_intrinsics, step_size);


	mat4f depth_intrinsics_inv = _depth_intrinsics;
	depth_intrinsics_inv.invert();

	for (unsigned int y = 0; y < _depth_sensor.getDepthHeight(); y += step_size) {
		for (unsigned int x = 0; x < _depth_sensor.getDepthWidth(); x += step_size) {
			float depth = _depth_images[frame][x + y * _depth_sensor.getDepthWidth()];
			if (depth != 0.) {
				vec3f p(static_cast<float>(x), static_cast<float>(y), 1.);
				p = depth_intrinsics_inv * p;
				depth = (350.f - depth) / 1000.f;
				p = p * depth;
				points.push_back(p);
			}
		}
	}
	return points;
}


PointsFromDepthData::PointsFromDepthData(DepthSensor & depth_sensor,
										 mat4f color_intrinsics,
										 mat4f depth_intrinsics)
	: _depth_sensor(depth_sensor)
	, _color_intrinsics(color_intrinsics)
	, _depth_intrinsics(depth_intrinsics)
{
}