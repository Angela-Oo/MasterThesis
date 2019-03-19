#include "showRegisterTwoRigideFrames.h"
#include <numeric>
#include "ext-depthcamera/calibratedSensorData.h"
#include "algo/icp.h"

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


void ShowTwoRigideRegisteredFrames::renderPoints(std::vector<ml::vec3f> points_frame_A, std::vector<ml::vec3f> points_frame_B)
{
	ml::mat4f rotation = ml::mat4f::rotationX(90.) * ml::mat4f::rotationY(180.);
	ml::mat4f transform = ml::mat4f::translation({ -0.5f, -2.f, 1.2f });
	ml::mat4f depth_extrinsics = transform * rotation;// *scale;
	std::for_each(points_frame_A.begin(), points_frame_A.end(), [&](ml::vec3f & p) { p = depth_extrinsics * p; });
	std::for_each(points_frame_B.begin(), points_frame_B.end(), [&](ml::vec3f & p) { p = depth_extrinsics * p; });


	std::vector<ml::vec4f> color_frame_A(points_frame_A.size());
	std::fill(color_frame_A.begin(), color_frame_A.end(), ml::RGBColor::Orange);
	m_pointCloudFrameA.init(*_graphics, ml::meshutil::createPointCloudTemplate(ml::Shapesf::box(0.001f), points_frame_A, color_frame_A));

	std::vector<ml::vec4f> color_frame_B(points_frame_B.size());
	std::fill(color_frame_B.begin(), color_frame_B.end(), ml::RGBColor::Green);
	m_pointCloudFrameB.init(*_graphics, ml::meshutil::createPointCloudTemplate(ml::Shapesf::box(0.001f), points_frame_B, color_frame_B));
}

void ShowTwoRigideRegisteredFrames::init(ml::ApplicationData &app)
{
	_transformation = ml::mat4f::identity();

	_graphics = &app.graphics;
	m_shaderManager.init(app.graphics);
	m_shaderManager.registerShader("shaders/pointCloud.hlsl", "pointCloud");
	m_constants.init(app.graphics);
	//configImageReaderSensor("C:/Users/Angela/Meins/Studium/MasterThesis/data/sokrates-ps/");
	configImageReaderSensor("D:/Studium/MasterThesis/input_data/sokrates-ps/");

	
	_sensor_data = std::make_unique<CalibrateSensorDataWrapper>(_depth_sensor,
																_depth_sensor.getDepthIntrinsics(), ml::mat4f::identity(),
																_depth_sensor.getColorIntrinsics(), ml::mat4f::identity());

	for(int i = 0; i < 6; i++)
		_sensor_data->processFrame();
	
	_points_a = _sensor_data->getPoints(0, 10);
	_points_b = _sensor_data->getPoints(5, 10);

	float scale_factor = 0.004;
	ml::mat4f scale = ml::mat4f::scale({ scale_factor, scale_factor, scale_factor });
	std::for_each(_points_a.begin(), _points_a.end(), [&](ml::vec3f & p) { p = scale * p; });
	std::for_each(_points_b.begin(), _points_b.end(), [&](ml::vec3f & p) { p = scale * p; });

	renderPoints(_points_a, _points_b);

	_points_a_icp = _points_a;
	_points_b_icp = _points_b;

	auto translation = ml::mat4f::translation({ 1.f, 0., 0. });
	std::for_each(_points_a_icp.begin(), _points_a_icp.end(), [&](ml::vec3f & p) { p = translation * p; });
	std::for_each(_points_b_icp.begin(), _points_b_icp.end(), [&](ml::vec3f & p) { p = (translation * _transformation) * p; });
}



void ShowTwoRigideRegisteredFrames::icp()
{
	_transformation = iterative_closest_points(_points_a_icp, _points_b_icp);

	std::for_each(_points_a_icp.begin(), _points_a_icp.end(), [&](ml::vec3f & p) { p = _transformation * p; });

	auto render_points_a = _points_a_icp;
	auto render_points_b = _points_b_icp;
	render_points_a.insert(render_points_a.end(), _points_a.begin(), _points_a.end());
	render_points_b.insert(render_points_b.end(), _points_b.begin(), _points_b.end());

	renderPoints(render_points_a, render_points_b);
}


void ShowTwoRigideRegisteredFrames::icptest()
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
		_icp_nn = std::make_unique<ICP>(_points_a_icp, _points_b_icp, options);
	//}

	//if (!_icp_nn->finished()) {
		//_transformation = _icp_nn->solveIteration();
		_transformation = _icp_nn->solve();

		auto render_points_a = _points_a_icp;
		auto render_points_b = _points_b_icp;

		std::for_each(render_points_a.begin(), render_points_a.end(), [&](ml::vec3f & p) { p = _transformation * p; });

		render_points_a.insert(render_points_a.end(), _points_a.begin(), _points_a.end());
		render_points_b.insert(render_points_b.end(), _points_b.begin(), _points_b.end());

		renderPoints(render_points_a, render_points_b);
	}
}

void ShowTwoRigideRegisteredFrames::render(ml::Cameraf& camera)
{
	if (icp_active) {
		icptest();
		icp_active = false;
	}

	ConstantBuffer constants;
	constants.worldViewProj = camera.getViewProj();

	m_constants.updateAndBind(constants, 0);
	m_shaderManager.bindShaders("pointCloud");
	m_constants.bind(0);
	m_pointCloudFrameA.render();
	m_pointCloudFrameB.render();
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

