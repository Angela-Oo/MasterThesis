#include "stdafx.h"
#include "showRegisterTwoRigideFrames.h"
#include <numeric>
#include "ext-depthcamera/calibratedSensorData.h"
#include "algo/icp.h"
#include "algo/non_rigid_deformation.h"
#include "algo/se3.h"


void RigidRegistration::solve()
{
	_transformation = iterative_closest_points(_points_a, _points_b);
}


void RigidRegistration::icp_calc_nn_in_cost_function()
{
	if (!_icp_nn) {
		ceres::Solver::Options options;
		options.sparse_linear_algebra_library_type = ceres::EIGEN_SPARSE;
		options.minimizer_type = ceres::MinimizerType::TRUST_REGION;
		options.trust_region_strategy_type = ceres::TrustRegionStrategyType::LEVENBERG_MARQUARDT;
		options.line_search_direction_type = ceres::LineSearchDirectionType::LBFGS;
		options.linear_solver_type = ceres::LinearSolverType::SPARSE_NORMAL_CHOLESKY;
		options.preconditioner_type = ceres::PreconditionerType::JACOBI;// SCHUR_JACOBI;
		options.max_num_iterations = 50;
		options.logging_type = ceres::LoggingType::SILENT;
		options.minimizer_progress_to_stdout = false;
		_icp_nn = std::make_unique<ICP>(_points_a, _points_b, options);

		//if (!_icp_nn->finished())
			//_transformation = _icp_nn->solveIteration();
		_transformation = _icp_nn->solve();
	}
}

std::vector<ml::vec3f> RigidRegistration::getPointsA()
{
	auto transformed_points = _points_a;
	std::for_each(transformed_points.begin(), transformed_points.end(), [&](ml::vec3f & p) { p = _transformation * p; });
	return transformed_points;
}

std::vector<ml::vec3f> RigidRegistration::getPointsB()
{
	return _points_b;
}

std::vector<ml::vec3f> RigidRegistration::getPointsDeformationGraph()
{
	return std::vector<ml::vec3f>();
}

RigidRegistration::RigidRegistration(const std::vector<ml::vec3f> & points_a, const std::vector<ml::vec3f> & points_b)
	: _points_a(points_a)
	, _points_b(points_b)
	, _transformation(ml::mat4f::identity())
{}

//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------

void NonRigidRegistration::solve()
{

	//AsRigidAsPossible arap(_points_a, _points_b, options);
	//_points_b = arap.solve();
	if (!_embedded_deformation) {
		ceres::Solver::Options options;
		options.sparse_linear_algebra_library_type = ceres::EIGEN_SPARSE;
		options.minimizer_type = ceres::MinimizerType::TRUST_REGION;
		options.trust_region_strategy_type = ceres::TrustRegionStrategyType::LEVENBERG_MARQUARDT;
		options.line_search_direction_type = ceres::LineSearchDirectionType::LBFGS;
		options.linear_solver_type = ceres::LinearSolverType::SPARSE_NORMAL_CHOLESKY;
		options.preconditioner_type = ceres::PreconditionerType::JACOBI;// SCHUR_JACOBI;
		options.max_num_iterations = 50;
		options.logging_type = ceres::LoggingType::SILENT;
		options.minimizer_progress_to_stdout = false;

		_embedded_deformation = std::make_unique<EmbeddedDeformation>(_points_a, _points_b, options);
	}
	if (!_embedded_deformation->finished()) {
		_embedded_deformation->solveIteration();
		_points_a = _embedded_deformation->getDeformedPoints();
	}
}


std::vector<ml::vec3f> NonRigidRegistration::getPointsA()
{
	return _points_a;
}

std::vector<ml::vec3f> NonRigidRegistration::getPointsB()
{
	return _points_b;
}

std::vector<ml::vec3f> NonRigidRegistration::getPointsDeformationGraph()
{
	if (_embedded_deformation)
		return _embedded_deformation->getDeformationGraph();
	else
		return std::vector<ml::vec3f>();
}

NonRigidRegistration::NonRigidRegistration()
	: _transformation()
{
	for (int i = 0; i < 50; i++) {
		float x = 0.01 *static_cast<float>(i);
		_points_a.push_back({ x,0.,0. });
	}
	//_points_a.push_back({ 0.05,0.,0. });
	//_points_a.push_back({ 0.1,0.,0. });
	//_points_a.push_back({ 0.15,0.,0. });
	//_points_a.push_back({ 0.2,0.,0. });
	//_points_a.push_back({ 0.25,0.,0. });

	_points_b = _points_a;
	for (int i = 25; i < 50; i++) {
		float z = 0.005 *static_cast<double>(i-24);
		_points_b[i].y = z;
	}
	//_points_b[5].y = 0.1;
	//_points_b[5].x = 0.2;
}

NonRigidRegistration::NonRigidRegistration(const std::vector<ml::vec3f> & points_a, const std::vector<ml::vec3f> & points_b)
	: _points_a(points_a)
	, _points_b(points_b)
{}

//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------


void ShowTwoRigideRegisteredFrames::configImageReaderSensor(std::string filepath)
{
	_depth_sensor.setBaseFilePath(filepath);
	auto frame_number = [](unsigned int idx) {
		char frameNumber_c[10];
		sprintf_s(frameNumber_c, "%06d", idx);
		return std::string(frameNumber_c);
	};
	_depth_sensor.setDepthFileName([&frame_number](unsigned int idx) { return "frame-" + frame_number(idx) + ".depth.png"; });
	_depth_sensor.setColorFileName([&frame_number](unsigned int idx) { return "frame-" + frame_number(idx) + ".color.png"; });
	_depth_sensor.setDepthIntrinsicsFileName("depthIntrinsics.txt");
	_depth_sensor.setColorIntrinsicsFileName("colorIntrinsics.txt");

	_depth_sensor.createFirstConnected();
	_depth_sensor.setNumFrames(56);
	_depth_sensor.toggleNearMode();
}

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
	std::vector<ml::vec4f> color_frame_A(render_points_a.size());
	std::fill(color_frame_A.begin(), color_frame_A.end(), ml::RGBColor::Yellow);
	m_pointCloudFrameA.init(*_graphics, ml::meshutil::createPointCloudTemplate(ml::Shapesf::box(0.001f), render_points_a, color_frame_A));

	std::vector<ml::vec4f> color_frame_B(render_points_b.size());
	std::fill(color_frame_B.begin(), color_frame_B.end(), ml::RGBColor::Green);
	m_pointCloudFrameB.init(*_graphics, ml::meshutil::createPointCloudTemplate(ml::Shapesf::box(0.001f), render_points_b, color_frame_B));

	std::vector<ml::vec4f> color_frame_dg(render_points_dg.size());
	std::fill(color_frame_dg.begin(), color_frame_dg.end(), ml::RGBColor::Blue);
	m_pointCloudFrameDG.init(*_graphics, ml::meshutil::createPointCloudTemplate(ml::Shapesf::box(0.001f), render_points_dg, color_frame_dg));
}

void ShowTwoRigideRegisteredFrames::initICP()
{
	//configImageReaderSensor("C:/Users/Angela/Meins/Studium/MasterThesis/data/sokrates-ps/");
	configImageReaderSensor("D:/Studium/MasterThesis/input_data/sokrates-ps/");

	_sensor_data = std::make_unique<CalibrateSensorDataWrapper>(_depth_sensor,
																_depth_sensor.getDepthIntrinsics(), ml::mat4f::identity(),
																_depth_sensor.getColorIntrinsics(), ml::mat4f::identity());

	for (int i = 0; i < 6; i++)
		_sensor_data->processFrame();

	_points_a = _sensor_data->getPoints(0, 4);
	_points_b = _sensor_data->getPoints(5, 4);

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
	_graphics = &app.graphics;
	m_shaderManager.init(app.graphics);
	m_shaderManager.registerShader("shaders/pointCloud.hlsl", "pointCloud");
	m_constants.init(app.graphics);	

	//initICP();
	initNonRigidRegistration();

	renderPoints();
}


void ShowTwoRigideRegisteredFrames::render(ml::Cameraf& camera)
{
	if (icp_active && _registration) {
		_registration->solve();
		renderPoints();
		icp_active = false;
	}

	ConstantBuffer constants;
	constants.worldViewProj = camera.getViewProj();

	m_constants.updateAndBind(constants, 0);
	m_shaderManager.bindShaders("pointCloud");
	m_constants.bind(0);
	m_pointCloudFrameA.render();
	m_pointCloudFrameB.render();
	m_pointCloudFrameDG.render();
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

