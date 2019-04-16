#pragma once
#include "../mLibInclude.h"
#include "i_showData.h"
#include "algo/registration.h"
#include "input_reader/depth_image_reader.h"
#include "pointsRenderer.h"

class ShowTwoRigideRegisteredFrames : public IShowData
{
private:
	std::vector<ml::vec3f> _points_a;
	std::vector<ml::vec3f> _points_b;

	std::unique_ptr<IReader> _reader;
	std::unique_ptr<PointsRenderer> _point_renderer;
	bool icp_active = false;
	std::unique_ptr<IRegistration> _registration;
private:
	void transform(std::vector<ml::vec3f>& points);
	void renderPoints();
	void initICP();
	void initNonRigidRegistration();
public:
	void init(ml::ApplicationData &app) override;
	void render(ml::Cameraf& camera) override;
	void key(UINT key) override;
public:
	ShowTwoRigideRegisteredFrames() {};
	~ShowTwoRigideRegisteredFrames() = default;
};