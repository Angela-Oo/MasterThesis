#include "PointsFromDepthSensor.h"


PointCloudf PointsFromDepthData::getPoints()
{
	_depth_sensor.processDepth();
	_depth_sensor.processColor();

	std::vector<unsigned short> depth_data;
	for (unsigned int i = 0; i < _depth_sensor.getDepthHeight(); i++) {
		for (unsigned int j = 0; j < _depth_sensor.getDepthWidth(); j++) {
			float depth = _depth_sensor.getDepth(j, i);
			if (depth != 0.)
				depth = 200 - depth;
			depth_data.push_back(static_cast<unsigned short>(depth));
		}
	}

	//DepthImage32 depth_image(reader.getDepthWidth(), reader.getDepthHeight(), depth_data.data());
	auto color_rgbx = _depth_sensor.getColorRGBX();
	std::vector<vec3uc> color_data;
	for (unsigned int i = 0; i < _depth_sensor.getColorHeight(); i++) {
		for (unsigned int j = 0; j < _depth_sensor.getColorWidth(); j++) {
			const unsigned int idx = (i * _depth_sensor.getColorWidth() + j) * 4;	//4 bytes per entry
			color_data.push_back(vec3uc(color_rgbx[idx + 0], color_rgbx[idx + 1], color_rgbx[idx + 2]));
		}
	}

	SensorData::CalibrationData calibrationColor(_color_intrinsics);
	SensorData::CalibrationData calibrationDepth(_depth_intrinsics);
	SensorData data;
	data.initDefault(_depth_sensor.getColorWidth(), _depth_sensor.getColorHeight(),
					 _depth_sensor.getDepthWidth(), _depth_sensor.getDepthHeight(),
					 calibrationColor, calibrationDepth);

	data.addFrame(color_data.data(), depth_data.data());
	return data.computePointCloud(0);
}

PointsFromDepthData::PointsFromDepthData(DepthSensor & depth_sensor,
										 mat4f color_intrinsics,
										 mat4f depth_intrinsics)
	: _depth_sensor(depth_sensor)
	, _color_intrinsics(color_intrinsics)
	, _depth_intrinsics(depth_intrinsics)
{
}