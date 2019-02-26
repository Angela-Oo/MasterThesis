#pragma once
#include "../mLibInclude.h"
#include "i_showData.h"
#include "PointsFromDepthSensor.h"
#include "kinect/ImageReaderSensor.h"



class RenderMesh : public IShowData
{
public:
	void init(ml::ApplicationData &app) override;
	void render(ml::Cameraf& camera) override;
private:
	ml::D3D11TriMesh m_mesh;
	ml::D3D11ShaderManager m_shaderManager;
	ml::D3D11ConstantBuffer<ConstantBuffer> m_constants;
	ml::D3D11Buffer<vec4f> m_buffer;
};

//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------


class ShowRGBDImageData : public IShowData
{
private:
	std::vector<vec3f> initImagePoints();
	std::vector<vec3f> initSokratesPoints();
	std::vector<vec3f> processFrame();
public:
	void init(ml::ApplicationData &app) override;
	void render(ml::Cameraf& camera) override;
private:
	ml::D3D11TriMesh m_pointCloud;
	ml::D3D11ShaderManager m_shaderManager;
	ml::D3D11ConstantBuffer<ConstantBuffer> m_constants;
	std::unique_ptr<PointsFromDepthData> _rgbd_frame_to_point_cloud;
	ImageReaderSensor _reader;
	//ml::GraphicsDevice & _graphics;
public:
	ShowRGBDImageData() {};
	~ShowRGBDImageData() = default;
};