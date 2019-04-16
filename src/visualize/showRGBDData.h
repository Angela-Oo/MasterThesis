#pragma once
#include "../mLibInclude.h"
#include "i_showData.h"
#include "kinect/SensorDataWrapper.h"
#include "kinect/ImageReaderSensor.h"
#include "input_reader/i_reader.h"
#include "pointsRenderer.h"


class RenderMesh : public IShowData
{
public:
	void init(ml::ApplicationData &app) override;
	void render(ml::Cameraf& camera) override;
	void key(UINT key) override {};
private:
	ml::D3D11TriMesh m_mesh;
	ml::D3D11ShaderManager m_shaderManager;
	ml::D3D11ConstantBuffer<ConstantBuffer> m_constants;
	ml::D3D11Buffer<ml::vec4f> m_buffer;
};

//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------


class ShowRGBDImageData : public IShowData
{
public:
	void init(ml::ApplicationData &app) override;
	void render(ml::Cameraf& camera) override;
	void key(UINT key) override {};
private:
	std::unique_ptr<IReader> _reader;
	std::unique_ptr<PointsRenderer> _point_renderer;
public:
	ShowRGBDImageData() = default;
	~ShowRGBDImageData() = default;
};