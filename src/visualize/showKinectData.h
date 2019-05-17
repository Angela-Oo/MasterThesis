#pragma once
#include "../mLibInclude.h"
#include "kinect/SensorDataWrapper.h"
#include "i_showData.h"
#include "pointsRenderer.h"
#include "kinect/PrimeSenseSensor.h"
#include "algo/registration.h"
#include "input_reader/i_reader.h"
#include <chrono>
#include "algo/registration/i_registration.h"

class ShowKinectData : public IShowData
{
private:
	void icp(int frame_a, int frame_b);
	void non_rigid_registration(int frame_a, int frame_b);
	void renderPoints(int frame);
	void renderRegisteredPoints();
public:
	void init(ml::ApplicationData &app) override;
	void render(ml::Cameraf& camera) override;
	void key(UINT key) override;
private:
	std::unique_ptr<IReader> _reader;
	std::unique_ptr<PointsRenderer> _point_renderer;
	std::unique_ptr<IRegistration> _registration;
	unsigned int _current_frame = 0;
	std::vector<unsigned int> _selected_frame_for_registration;
	std::chrono::time_point<std::chrono::system_clock> _start_time;
	bool _record_frames = false;
	bool _solve_non_rigid_registration = false;
};