#pragma once
#include "../mLibInclude.h"
#include "i_showData.h"
#include "PointsFromDepthSensor.h"
#include "kinect/ImageReaderSensor.h"

class ShowTwoRigideRegisteredFrames : public IShowData
{
private:
	void configImageReaderSensor(std::string filepath);
	std::vector<vec3f> processFrame();
	void renderPoints(std::vector<vec3f> & points_frame_A, std::vector<vec3f> & points_frame_B);
public:
	void init(ml::ApplicationData &app) override;
	void render(ml::Cameraf& camera) override;
private:
	ml::D3D11TriMesh m_pointCloudFrameA;
	ml::D3D11TriMesh m_pointCloudFrameB;
	ml::D3D11ShaderManager m_shaderManager;
	ml::D3D11ConstantBuffer<ConstantBuffer> m_constants;
	std::unique_ptr<SensorDataWrapper> _rgbd_frame_to_point_cloud;
	ImageReaderSensor _depth_sensor;
	ml::GraphicsDevice * _graphics;
public:
	ShowTwoRigideRegisteredFrames() {};
	~ShowTwoRigideRegisteredFrames() = default;
};