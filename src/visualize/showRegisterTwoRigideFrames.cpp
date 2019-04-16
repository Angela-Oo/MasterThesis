#include "stdafx.h"
#include "showRegisterTwoRigideFrames.h"
#include <numeric>
#include "ext-depthcamera/calibratedSensorData.h"
#include "algo/registration.h"
#include "algo/se3.h"
#include "input_reader/depth_image_reader.h"
//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------


void ShowTwoRigideRegisteredFrames::transform(std::vector<ml::vec3f>& points)
{
	ml::mat4f rotation = ml::mat4f::rotationX(90.) * ml::mat4f::rotationY(180.);
	ml::mat4f transform = ml::mat4f::translation({ -0.5f, -2.f, 1.2f });
	ml::mat4f depth_extrinsics = transform * rotation;
	std::for_each(points.begin(), points.end(), [&](ml::vec3f & p) { p = depth_extrinsics * p; });
}

void ShowTwoRigideRegisteredFrames::renderPoints()
{
	std::vector<ml::vec3f> render_points_a = _registration->getPointsA();
	std::vector<ml::vec3f> render_points_b = _registration->getPointsB();
	std::vector<ml::vec3f> render_points_dg = _registration->getPointsDeformationGraph();

	render_points_a.insert(render_points_a.end(), _points_a.begin(), _points_a.end());
	render_points_b.insert(render_points_b.end(), _points_b.begin(), _points_b.end());

	// transform
	transform(render_points_a);
	transform(render_points_b);
	transform(render_points_dg);

	// render point clouds
	_point_renderer->insertPoints("frame_deformation_graph", render_points_dg, ml::RGBColor::Blue, 0.002f);
	_point_renderer->insertPoints("frame_registered_A", render_points_a, ml::RGBColor::Orange);
	_point_renderer->insertPoints("frame_registered_B", render_points_b, ml::RGBColor::Green);
}

void ShowTwoRigideRegisteredFrames::initICP()
{
	_reader = std::make_unique<DepthImageReader>("D:/Studium/MasterThesis/input_data/sokrates-ps/");
	

	for (int i = 0; i < 6; i++)
		_reader->processFrame();

	_points_a = _reader->getPoints(0);
	_points_b = _reader->getPoints(4);

	float scale_factor = 0.004;
	ml::mat4f scale = ml::mat4f::scale({ scale_factor, scale_factor, scale_factor });
	std::for_each(_points_a.begin(), _points_a.end(), [&](ml::vec3f & p) { p = scale * p; });
	std::for_each(_points_b.begin(), _points_b.end(), [&](ml::vec3f & p) { p = scale * p; });

	auto points_a_icp = _points_a;
	auto points_b_icp = _points_b;

	auto translation = ml::mat4f::translation({ 1.f, 0., 0. });
	std::for_each(points_a_icp.begin(), points_a_icp.end(), [&](ml::vec3f & p) { p = translation * p; });
	std::for_each(points_b_icp.begin(), points_b_icp.end(), [&](ml::vec3f & p) { p = translation * p; });

	//_registration = std::make_unique<RigidRegistration>(points_a_icp, points_b_icp);
	_registration = std::make_unique<NonRigidRegistration>(points_a_icp, points_b_icp);
}


void ShowTwoRigideRegisteredFrames::initNonRigidRegistration()
{
	_registration = std::make_unique<NonRigidRegistration>();
}

void ShowTwoRigideRegisteredFrames::init(ml::ApplicationData &app)
{
	_point_renderer = std::make_unique<PointsRenderer>(app);
	
	initICP();
	//initNonRigidRegistration();

	renderPoints();
}


void ShowTwoRigideRegisteredFrames::render(ml::Cameraf& camera)
{
	if (icp_active && _registration) {
		_registration->solve();
		renderPoints();
		icp_active = false;
	}

	_point_renderer->render(camera);
}


void ShowTwoRigideRegisteredFrames::key(UINT key)
{
	if (key == KEY_I) {
		icp_active = true;
	}
	if (key == KEY_J) {
		icp_active = false;
	}
}

