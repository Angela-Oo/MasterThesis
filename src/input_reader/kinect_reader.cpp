#include "kinect_reader.h"
#include "kinect/KinectSensor.h"
#include "ext-depthcamera/sensorData.h"

ml::mat4f KinectReader::getWorldTransformation()
{
	float scale_factor = 2.0;
	ml::mat4f scale = ml::mat4f::scale({ scale_factor, scale_factor, scale_factor });
	ml::mat4f rotation = ml::mat4f::rotationY(180.) * ml::mat4f::rotationX(90.);
	ml::mat4f transform = ml::mat4f::translation({ -1.0f, -0.5f, 1.5f });
	return transform * rotation * scale;
}

std::vector<ml::vec3f> KinectReader::getPoints(unsigned int frame, unsigned int step_size)
{
	return _sensor_data_wrapper->getPoints(frame, step_size);
}

void KinectReader::processFrame()
{
	_sensor_data_wrapper->processFrame();
}

unsigned int KinectReader::frame()
{
	return _sensor_data_wrapper->getNumberFrames() -1;
}


void KinectReader::load(std::string filename)
{
	std::cout << "load recorded frames from .sens file " << filename << std::endl;
	std::ifstream input_file;
	input_file.open(filename);
	//input_file >> _sensor_data_wrapper->_sensor_data;
	//auto frame = _sensor_data_wrapper->_sensor_data.m_frames.size() - 1;
	//auto points = _sensor_data_wrapper->getPoints(frame);
	//m_pointCloud.init(*_graphics, ml::meshutil::createPointCloudTemplate(ml::Shapesf::box(0.002f), points /*_all_points*/));
}

void KinectReader::save(std::string filename)
{
	std::cout << "save recorded frames as .sens file " << filename << std::endl;
	std::ofstream output_file;
	output_file.open(filename);
	//_sensor_data_wrapper->_sensor_data.saveToFile(file);
	//_sensor_data_wrapper->_sensor_data.savePointCloud(file, 0);// .saveToFile(file);
	//output_file << _sensor_data_wrapper->_sensor_data;
}

KinectReader::KinectReader()
{
	if (_depth_sensor.createFirstConnected() == S_OK)
	{
		auto intrinsic = _depth_sensor.getIntrinsics();
		auto depth_intrinsics = intrinsic.converToMatrix();
		auto color_intrinsics = intrinsic.converToMatrix();
		ml::mat4f depth_extrinsics = getWorldTransformation();
		auto color_extrinsics = ml::mat4f::identity();

		_sensor_data_wrapper = std::make_unique<CalibrateSensorDataWrapper>(_depth_sensor,
																			depth_intrinsics, depth_extrinsics,
																			color_intrinsics, color_extrinsics);

		//_sensor_data_wrapper = std::make_unique<SensorDataWrapper>(_depth_sensor,
		//														   depth_intrinsics,
		//														   color_intrinsics);
		_sensor_data_wrapper->processFrame();
	}
	else {
		std::cout << "could not connect to camera" << std::endl;
	}
}
