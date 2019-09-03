#pragma once
#include "../mLibInclude.h"
#include "i_showData.h"
#include "input_reader/i_reader.h"
#include "algo/registration/interface/i_registration.h"
#include "render/points_renderer.h"

class ShowTwoRigideRegisteredFrames : public IShowData
{
private:
	std::unique_ptr<IReader> _reader;
	std::unique_ptr<PointsRenderer> _point_renderer;
	bool icp_active = false;
	std::unique_ptr<Registration::IRegistration> _registration;
private:
	void renderPoints();
	void initRegistration();
	void initReader();
public:
	void init(ml::ApplicationData &app) override;
	void render(ml::Cameraf& camera) override;
	void key(UINT key) override;
public:
	ShowTwoRigideRegisteredFrames() {};
	~ShowTwoRigideRegisteredFrames() = default;
};