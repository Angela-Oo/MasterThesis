#pragma once
#include "../mLibInclude.h"
#include "i_showData.h"
#include "kinect/SensorDataWrapper.h"
#include "kinect/ImageReaderSensor.h"

class ShowTwoRigideRegisteredFrames : public IShowData
{
private:
	void configImageReaderSensor(std::string filepath);
	//std::vector<ml::vec3f> processFrame();
	void renderPoints(std::vector<ml::vec3f> points_frame_A, std::vector<ml::vec3f> points_frame_B);
	void icp();
public:
	void init(ml::ApplicationData &app) override;
	void render(ml::Cameraf& camera) override;
	void key(UINT key) override;
private:
	std::vector<ml::vec3f> _points_a;
	std::vector<ml::vec3f> _points_b;
	std::vector<ml::vec3f> _points_a_icp;
	std::vector<ml::vec3f> _points_b_icp;
	ml::mat4f _transformation;

	ml::D3D11TriMesh m_pointCloudFrameA;
	ml::D3D11TriMesh m_pointCloudFrameB;
	ml::D3D11ShaderManager m_shaderManager;
	ml::D3D11ConstantBuffer<ConstantBuffer> m_constants;
	std::unique_ptr<SensorDataWrapper> _rgbd_frame_to_point_cloud;
	std::unique_ptr<CalibrateSensorDataWrapper> _sensor_data;
	ImageReaderSensor _depth_sensor;
	ml::GraphicsDevice * _graphics;
	bool icp_active = false;
public:
	ShowTwoRigideRegisteredFrames() {};
	~ShowTwoRigideRegisteredFrames() = default;
};