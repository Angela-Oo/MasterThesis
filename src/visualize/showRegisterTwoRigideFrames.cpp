#include "stdafx.h"
#include "showRegisterTwoRigideFrames.h"
#include <numeric>
#include "ext-depthcamera/calibratedSensorData.h"
#include "algo/registration.h"
#include "input_reader/depth_image_reader.h"


void ShowTwoRigideRegisteredFrames::renderPoints()
{
	auto render_points_a = _registration->getDeformedPoints();
	auto render_points_b = _registration->getTarget();
	//std::vector<ml::vec3f> render_points_dg = _registration->getPointsDeformationGraph();

	// render point clouds
	//_point_renderer->insertPoints("frame_deformation_graph", render_points_dg, ml::RGBColor::Blue, 0.002f);
	_point_renderer->insertPoints("frame_registered_A", render_points_a, ml::RGBColor::Orange);
	_point_renderer->insertPoints("frame_registered_B", render_points_b, ml::RGBColor::Green);
}

void ShowTwoRigideRegisteredFrames::initRegistration()
{	
	for (int i = 0; i < 6; i++)
		_reader->processFrame();

	auto points_a = _reader->getPoints(0);
	auto points_b = _reader->getPoints(4);

	_point_renderer->insertPoints("frame_A", points_a, ml::RGBColor::Orange);
	_point_renderer->insertPoints("frame_B", points_b, ml::RGBColor::Green);
	
	auto translation = ml::mat4f::translation({ 1.f, 0., 0. });
	std::for_each(points_a.begin(), points_a.end(), [&](ml::vec3f & p) { p = translation * p; });
	std::for_each(points_b.begin(), points_b.end(), [&](ml::vec3f & p) { p = translation * p; });

	//_registration = std::make_unique<RigidRegistration>(points_a_icp, points_b_icp);
	ml::TriMeshf mesh_a;
	for (auto & p : points_a)
		mesh_a.m_vertices.push_back(p);

	ml::TriMeshf mesh_b;
	for (auto & p : points_b)
		mesh_b.m_vertices.push_back(p);

	_registration = std::make_unique<ED::EmbeddedDeformation>(mesh_a, mesh_b, ceresOption());
}

void ShowTwoRigideRegisteredFrames::initReader()
{
	float scale_factor = 0.004;
	ml::mat4f scale = ml::mat4f::scale({ scale_factor, scale_factor, scale_factor });
	ml::mat4f rotation = ml::mat4f::rotationX(90.) * ml::mat4f::rotationY(180.);
	ml::mat4f transform = ml::mat4f::translation({ -0.5f, -2.f, 1.2f });
	ml::mat4f transformation_camera_extrinsics = transform * rotation * scale;

	_reader = std::make_unique<DepthImageReader>("D:/Studium/MasterThesis/input_data/sokrates-ps/", transformation_camera_extrinsics);
}

void ShowTwoRigideRegisteredFrames::init(ml::ApplicationData &app)
{
	_point_renderer = std::make_unique<PointsRenderer>(app);
	initReader();
	initRegistration();
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

