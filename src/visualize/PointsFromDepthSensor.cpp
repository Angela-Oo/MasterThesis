#include "PointsFromDepthSensor.h"

using namespace ml;


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
				//depth += 1.;
				//depth *= 1000.0;
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

std::vector<vec3f> SensorDataWrapper::get3DPoints(unsigned int step)
{
	std::vector<ml::vec3f> points;
	HRESULT hr = _depth_sensor.processDepth();
	if (hr == S_OK) {
		_depth_sensor.processColor();

		Intrinsics depth_intrinsic = _depth_sensor.getIntrinsics();
		ml::mat4f depth_intrinsics_inv = depth_intrinsic.converToMatrix();//_depth_intrinsics;
		depth_intrinsics_inv.invert();

		for (unsigned int i = 0; i < _depth_sensor.getDepthHeight(); i += step) {
			for (unsigned int j = 0; j < _depth_sensor.getDepthWidth(); j += step) {
				float depth = _depth_sensor.getDepth(j, i);
				if (depth != 0.) {
					vec3f p(static_cast<float>(j), static_cast<float>(i), 1.);
					p = depth_intrinsics_inv * p;
					//depth = (350.f - depth) / 1000.f;
					depth = (2. + depth);
					p = p * depth;
					points.push_back(p);
				}
			}
		}
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







void CalibrateSensorDataWrapper::processFrame()
{
	_depth_sensor.processDepth();
	_depth_sensor.processColor();

	// todo free memory
	float * depth_data = new float[_depth_sensor.getDepthWidth() * _depth_sensor.getDepthHeight()];
	for (unsigned int i = 0; i < _depth_sensor.getDepthHeight(); i++) {
		for (unsigned int j = 0; j < _depth_sensor.getDepthWidth(); j++) {
			float depth = _depth_sensor.getDepth(j, i);
			//if (depth != 0.)
			//	depth += 200.;
			//float depth = 10. * (1. + _depth_sensor.getDepth(j, i));
			//if (depth != 0.)
			//	depth += 1.;
			depth_data[i * _depth_sensor.getDepthWidth() + j] = (depth);
		}
	}
	auto color_rgbx = _depth_sensor.getColorRGBX();
	ml::vec4uc* color_data = new ml::vec4uc[_depth_sensor.getColorHeight() * _depth_sensor.getColorWidth()];
	for (unsigned int i = 0; i < _depth_sensor.getColorHeight(); i++) {
		for (unsigned int j = 0; j < _depth_sensor.getColorWidth(); j++) {
			const unsigned int idx = (i * _depth_sensor.getColorWidth() + j) * 4;	//4 bytes per entry
			color_data[i * _depth_sensor.getColorWidth() + j] = ml::vec4uc(color_rgbx[idx + 0], color_rgbx[idx + 1], color_rgbx[idx + 2], 0);
		}
	}

	_sensor_data.m_DepthImages.push_back(depth_data);
	_sensor_data.m_ColorImages.push_back(color_data);
	_sensor_data.m_DepthNumFrames++;
	_sensor_data.m_ColorNumFrames++;
}

std::vector<ml::vec3f> CalibrateSensorDataWrapper::getPoints(unsigned int frame, unsigned int step_size)
{
	std::vector<ml::vec3f> points;
	if (frame >= _sensor_data.m_DepthNumFrames)
		return points;
	
	int step = 4;
	for (unsigned int y = 0; y < _depth_sensor.getDepthHeight(); y += step_size) {
		for (unsigned int x = 0; x < _depth_sensor.getDepthWidth(); x += step_size) {
			auto point = _sensor_data.getWorldPos(x, y, frame);
			if (point != ml::vec3f::origin)
			{
				points.push_back(_sensor_data.m_CalibrationDepth.m_Extrinsic * point);
			}
		}
	}
	return points;
}

CalibrateSensorDataWrapper::CalibrateSensorDataWrapper(DepthSensor & depth_sensor,
													   ml::mat4f depth_intrinsics, ml::mat4f depth_extrinsics,
													   ml::mat4f color_intrinsics, ml::mat4f color_extrinsics)
	: _depth_sensor(depth_sensor)
{
	_sensor_data.m_CalibrationDepth.setMatrices(depth_intrinsics, depth_extrinsics);
	_sensor_data.m_CalibrationColor.setMatrices(color_intrinsics, color_extrinsics);
	_sensor_data.m_DepthImageWidth = _depth_sensor.getDepthWidth();
	_sensor_data.m_DepthImageHeight = _depth_sensor.getDepthHeight();
	_sensor_data.m_ColorImageWidth = _depth_sensor.getColorWidth();
	_sensor_data.m_ColorImageHeight = _depth_sensor.getColorHeight();
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