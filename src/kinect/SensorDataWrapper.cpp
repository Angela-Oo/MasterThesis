#include "SensorDataWrapper.h"

using namespace ml;



void SensorDataWrapper::processFrame()
{
	std::vector<unsigned short> depth_data(_depth_sensor.getDepthWidth() * _depth_sensor.getDepthHeight());
	std::vector<ml::vec3uc> color_data(_depth_sensor.getColorHeight() * _depth_sensor.getColorWidth());

	_depth_sensor.processFrame(depth_data.data(), color_data.data());
	_sensor_data.addFrame(color_data.data(), depth_data.data());
}

std::vector<ml::vec3f> SensorDataWrapper::getPoints(unsigned int frame, unsigned int step_size)
{
	std::vector<ml::vec3f> points;
	if (frame >= _sensor_data.m_frames.size())
		return points;
	

	//return _sensor_data.computePointCloud(frame).m_points;

	auto intrinsics = _depth_sensor._depth_sensor.getIntrinsics();
	auto intrinsic_matrix = intrinsics.converToMatrix();
	
	auto depth_image = _sensor_data.computeDepthImage(frame);
	return ::getPoints(depth_image, intrinsic_matrix, step_size);
}

SensorDataWrapper::SensorDataWrapper(DepthSensor & depth_sensor,
									 mat4f color_intrinsics,
									 mat4f depth_intrinsics)
	: _depth_sensor(depth_sensor)
{
	SensorData::CalibrationData calibrationColor(color_intrinsics);
	SensorData::CalibrationData calibrationDepth(depth_intrinsics);

	_sensor_data.initDefault(_depth_sensor.getColorWidth(), _depth_sensor.getColorHeight(),
							 _depth_sensor.getDepthWidth(), _depth_sensor.getDepthHeight(),
							 calibrationColor, calibrationDepth);
}







void CalibrateSensorDataWrapper::processFrame()
{
	float * depth_data = new float[_depth_sensor.getDepthWidth() * _depth_sensor.getDepthHeight()];
	ml::vec4uc* color_data = new ml::vec4uc[_depth_sensor.getColorHeight() * _depth_sensor.getColorWidth()];

	_depth_sensor.processFrame(depth_data, color_data);

	//		//if (depth != 0.)
	//		//	depth += 200.;
	//		//float depth = 10. * (1. + _depth_sensor.getDepth(j, i));
	//		//if (depth != 0.)
	//		//	depth += 1.;
	//		depth_data[i * _depth_sensor.getDepthWidth() + j] = (depth);


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
	for (unsigned int y = 0; y < _sensor_data.m_DepthImageHeight; y += step_size) {
		for (unsigned int x = 0; x < _sensor_data.m_DepthImageWidth; x += step_size) {
			auto point = _sensor_data.getWorldPos(x, y, frame);
			if (point != ml::vec3f::origin)
			{
				points.push_back(_sensor_data.m_CalibrationDepth.m_Extrinsic * point);
			}
		}
	}
	return points;
}

ml::SensorData CalibrateSensorDataWrapper::getSensorData()
{
	ml::SensorData data;
	//data.initDefault(_sensor_data.m_ColorImageWidth, 
	//				 _sensor_data.m_ColorImageHeight,
	//				 _sensor_data.m_DepthImageWidth,
	//				 _sensor_data.m_DepthImageHeight,
	//				 _sensor_data.m_CalibrationColor, 
	//				 _sensor_data.m_CalibrationDepth);

	for (int frame = 0; frame < _sensor_data.m_ColorNumFrames; frame++) {
		ml::vec4uc* sensor_color = _sensor_data.getColorFrame(frame);
		ml::vec3uc* color = new ml::vec3uc[_sensor_data.m_ColorImageWidth * _sensor_data.m_ColorImageHeight];
		for (int i = 0; i < _sensor_data.m_ColorImageWidth * _sensor_data.m_ColorImageHeight; i++)
			color[i] = ml::vec3uc(sensor_color[i][0], sensor_color[i][1], sensor_color[i][2]);

		float *sensor_depth = _sensor_data.getDepthFrame(frame);
		unsigned short *depth = new unsigned short[_sensor_data.m_DepthImageWidth * _sensor_data.m_DepthImageHeight];
		for (int i = 0; i < _sensor_data.m_DepthImageWidth * _sensor_data.m_DepthImageHeight; i++)
			depth[i] = static_cast<unsigned short>(sensor_depth[i] * 1000.);

		data.addFrame(color, depth);

		delete[] color;
		delete[] depth;
	}
	return data;
}

CalibrateSensorDataWrapper::CalibrateSensorDataWrapper(DepthSensor & depth_sensor,
													   ml::mat4f depth_intrinsics, ml::mat4f depth_extrinsics,
													   ml::mat4f color_intrinsics, ml::mat4f color_extrinsics)
	: _depth_sensor(depth_sensor)
{
	_sensor_data.m_CalibrationDepth.setMatrices(depth_intrinsics, depth_extrinsics);
	_sensor_data.m_CalibrationColor.setMatrices(color_intrinsics, color_extrinsics);
	_sensor_data.m_DepthImageWidth = _depth_sensor._depth_sensor.getDepthWidth();
	_sensor_data.m_DepthImageHeight = _depth_sensor._depth_sensor.getDepthHeight();
	_sensor_data.m_ColorImageWidth = _depth_sensor._depth_sensor.getColorWidth();
	_sensor_data.m_ColorImageHeight = _depth_sensor._depth_sensor.getColorHeight();
}


CalibrateSensorDataWrapper::~CalibrateSensorDataWrapper()
{
	for (auto & color : _sensor_data.m_ColorImages)
		delete[] color;
	for (auto & depth : _sensor_data.m_DepthImages)
		delete[] depth;
}


