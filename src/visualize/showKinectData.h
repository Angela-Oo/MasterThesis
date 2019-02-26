#pragma once
#include "../mLibInclude.h"
#include "PointsFromDepthSensor.h"
#include "i_showData.h"


class ShowKinectData : public IShowData
{
private:
	void initKinectPoints(ml::GraphicsDevice & graphics);	
public:
	void init(ml::ApplicationData &app) override;
	void render(ml::Cameraf& camera) override;
private:
	ml::D3D11TriMesh m_pointCloud;
	ml::D3D11ShaderManager m_shaderManager;
	ml::D3D11ConstantBuffer<ConstantBuffer> m_constants;
	std::unique_ptr<PointsFromDepthData> _rgbd_frame_to_point_cloud;
};