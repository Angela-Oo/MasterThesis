#include "PointsFromDepthSensor.h"



std::vector<vec3f> SensorDataWrapper::addFrame(unsigned int step)
{
	std::vector<ml::vec3f> points;
	HRESULT hr = _depth_sensor.processDepth();
	if (hr == S_OK) {
		_depth_sensor.processColor();

		std::vector<unsigned short> depth_data(_depth_sensor.getDepthHeight() * _depth_sensor.getDepthWidth());
		for (unsigned int i = 0; i < _depth_sensor.getDepthHeight(); i += step) {
			for (unsigned int j = 0; j < _depth_sensor.getDepthWidth(); j += step) {
				float depth = _depth_sensor.getDepth(j, i);
				if (depth != 0.)
					depth = 350 - depth;
				depth_data[i * _depth_sensor.getDepthWidth() + j] = static_cast<unsigned short>(depth);
			}
		}

		//DepthImage32 depth_image(reader.getDepthWidth(), reader.getDepthHeight(), depth_data.data());
		auto color_rgbx = _depth_sensor.getColorRGBX();
		std::vector<vec3uc> color_data(_depth_sensor.getDepthHeight() * _depth_sensor.getDepthWidth());
		for (unsigned int i = 0; i < _depth_sensor.getColorHeight(); i += step) {
			for (unsigned int j = 0; j < _depth_sensor.getColorWidth(); j += step) {
				const unsigned int idx = (i * _depth_sensor.getColorWidth() + j) * 4;	//4 bytes per entry
				color_data[i * _depth_sensor.getDepthWidth() + j] = vec3uc(color_rgbx[idx + 0], color_rgbx[idx + 1], color_rgbx[idx + 2]);;
			}
		}

		_data.addFrame(color_data.data(), depth_data.data());
		points = _data.computePointCloud(_data.m_frames.size() - 1).m_points;
	}
	return points;
}



SensorDataWrapper::SensorDataWrapper(DepthSensor & depth_sensor,
									 mat4f color_intrinsics,
									 mat4f depth_intrinsics)
	: _depth_sensor(depth_sensor)
{
	SensorData::CalibrationData calibrationColor(color_intrinsics);
	SensorData::CalibrationData calibrationDepth(depth_intrinsics);

	_data.initDefault(_depth_sensor.getColorWidth(), _depth_sensor.getColorHeight(),
					  _depth_sensor.getDepthWidth(), _depth_sensor.getDepthHeight(),
					  calibrationColor, calibrationDepth);
}











std::vector<vec3f> PointsFromDepthData::getPoints(unsigned int step)
{
	std::vector<ml::vec3f> points;
	HRESULT hr = _depth_sensor.processDepth();
	if (hr == S_OK) {
		_depth_sensor.processColor();

		mat4f depth_intrinsics_inv = _depth_intrinsics;
		depth_intrinsics_inv.invert();
		
		for (unsigned int i = 0; i < _depth_sensor.getDepthHeight(); i += step) {
			for (unsigned int j = 0; j < _depth_sensor.getDepthWidth(); j += step) {
				float depth = _depth_sensor.getDepth(j, i);
				if (depth != 0.) {
					vec3f p(static_cast<float>(j), static_cast<float>(i), 1.);
					p = depth_intrinsics_inv * p;
					depth = (350.f - depth) / 1000.f;
					p = p * depth;
					points.push_back(p);
				}
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