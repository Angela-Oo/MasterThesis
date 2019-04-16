#pragma once
#include "../mLibInclude.h"
#include "kinect/SensorDataWrapper.h"
#include "i_showData.h"
#include "kinect/PrimeSenseSensor.h"
#include <chrono>
#include "algo/registration.h"
#include "input_reader/i_reader.h"

class ShowKinectData : public IShowData
{
private:
	void renderPoints(int frame);
	void icp(int frame_a, int frame_b);
	void non_rigid_registration(int frame_a, int frame_b);
	void renderRegisteredPoints();
public:
	void init(ml::ApplicationData &app) override;
	void render(ml::Cameraf& camera) override;
	void key(UINT key) override;
private:
	std::unique_ptr<IReader> _reader;
	std::vector<ml::vec3f> _points_a;
	std::vector<ml::vec3f> _points_b;
	ml::D3D11TriMesh m_pointCloudFrameA;
	ml::D3D11TriMesh m_pointCloudFrameB;
	ml::D3D11TriMesh m_pointCloudFrameDG;
	std::unique_ptr<IRegistration> _registration;
	ml::D3D11ShaderManager m_shaderManager;
	ml::D3D11ConstantBuffer<ConstantBuffer> m_constants;
	ml::GraphicsDevice * _graphics;
	unsigned int _current_frame = 0;
	std::vector<unsigned int> _selected_frame_for_registration;
	std::chrono::time_point<std::chrono::system_clock> _start_time;
	std::vector<ml::vec3f> _all_points;

	bool _record_frames = false;
	bool _solve_non_rigid_registration = false;
};