#include "showKinectData.h"
//#include "kinect/BinaryDumpReader.h"
#include "kinect/KinectSensor.h"

#include "ext-depthcamera/sensorData.h"
#include <numeric>
#include "algo/rigid_registration/icp.h"
#include "algo/non_rigid_registration/embedded_deformation.h"
#include "algo/registration.h"


using namespace ml;


void ShowKinectData::renderPoints(int frame)
{
	auto points = _sensor_data_wrapper->getPoints(frame, 3);
	m_pointCloudFrameA.init(*_graphics, ml::meshutil::createPointCloudTemplate(ml::Shapesf::box(0.002f), points /*_all_points*/));
}

mat4f ShowKinectData::getWorldTransformation()
{
	float scale_factor = 2.0;
	mat4f scale = mat4f::scale({ scale_factor, scale_factor, scale_factor });
	mat4f rotation = mat4f::rotationY(180.) * mat4f::rotationX(90.);
	mat4f transform = mat4f::translation({ -1.0f, -0.5f, 1.5f });
	return transform * rotation * scale;
}

void ShowKinectData::init(ml::ApplicationData &app)
{
	_graphics = &app.graphics;
	_start_time = std::chrono::system_clock::now();

	if (_depth_sensor.createFirstConnected() == S_OK)
	{
		auto intrinsic = _depth_sensor.getIntrinsics();
		auto depth_intrinsics = intrinsic.converToMatrix();
		auto color_intrinsics = intrinsic.converToMatrix();
		mat4f depth_extrinsics = getWorldTransformation();
		auto color_extrinsics = ml::mat4f::identity();

		_sensor_data_wrapper = std::make_unique<CalibrateSensorDataWrapper>(_depth_sensor,
																			depth_intrinsics, depth_extrinsics,
																			color_intrinsics, color_extrinsics);

		//_sensor_data_wrapper = std::make_unique<SensorDataWrapper>(_depth_sensor,
		//														   depth_intrinsics,
		//														   color_intrinsics);
		_sensor_data_wrapper->processFrame();
		_frame++;
	}
	else {
		std::cout << "could not connect to camera" << std::endl;
	}

	m_shaderManager.init(app.graphics);
	m_shaderManager.registerShader("shaders/pointCloud.hlsl", "pointCloud");
	m_constants.init(app.graphics);
}

void ShowKinectData::render(ml::Cameraf& camera)
{
	if (_record_frames) {
		_sensor_data_wrapper->processFrame();
		renderPoints(_frame);
		_current_frame = _frame;
		_frame++;
	}
	else if (_registration && _selected_frame_for_registration.size() == 2) {
		non_rigid_registration(_selected_frame_for_registration[0], _selected_frame_for_registration[1]);
	}

	ConstantBuffer constants;
	constants.worldViewProj = camera.getViewProj();
	constants.modelColor = ml::vec4f(1.0f, 1.0f, 1.0f, 1.0f);

	m_constants.updateAndBind(constants, 0);
	m_shaderManager.bindShaders("pointCloud");
	m_constants.bind(0);
	m_pointCloudFrameA.render();
	m_pointCloudFrameB.render();
}



void ShowKinectData::icp(int frame_a, int frame_b)
{
	auto points_a = _sensor_data_wrapper->getPoints(frame_a);
	auto points_b = _sensor_data_wrapper->getPoints(frame_b);

	ceres::Solver::Options options;
	options.sparse_linear_algebra_library_type = ceres::EIGEN_SPARSE;
	options.minimizer_type = ceres::MinimizerType::TRUST_REGION;
	options.trust_region_strategy_type = ceres::TrustRegionStrategyType::LEVENBERG_MARQUARDT;
	options.max_num_iterations = 50;
	options.logging_type = ceres::LoggingType::SILENT;
	options.minimizer_progress_to_stdout = false;
	options.line_search_direction_type = ceres::LineSearchDirectionType::LBFGS;
	options.linear_solver_type = ceres::LinearSolverType::SPARSE_NORMAL_CHOLESKY;
	options.preconditioner_type = ceres::PreconditionerType::JACOBI;

	ICPNN icpnn(points_a, points_b, options);
	ml::mat4f transformation = icpnn.solve();

	std::for_each(points_a.begin(), points_a.end(), [&](ml::vec3f & p) { p = transformation * p; });

	std::vector<ml::vec4f> color_frame_A(points_a.size());
	std::fill(color_frame_A.begin(), color_frame_A.end(), ml::RGBColor::Orange);
	m_pointCloudFrameA.init(*_graphics, ml::meshutil::createPointCloudTemplate(ml::Shapesf::box(0.001f), points_a, color_frame_A));

	std::vector<ml::vec4f> color_frame_B(points_b.size());
	std::fill(color_frame_B.begin(), color_frame_B.end(), ml::RGBColor::Green);
	m_pointCloudFrameB.init(*_graphics, ml::meshutil::createPointCloudTemplate(ml::Shapesf::box(0.001f), points_b, color_frame_B));
}

void ShowKinectData::non_rigid_registration(int frame_a, int frame_b)
{
	if (!_registration) {
		_points_a = _sensor_data_wrapper->getPoints(frame_a);
		_points_b = _sensor_data_wrapper->getPoints(frame_b);

		auto points_a_icp = _points_a;
		auto points_b_icp = _points_b;

		auto translation = ml::mat4f::translation({ 1.f, 0., 0. });
		std::for_each(points_a_icp.begin(), points_a_icp.end(), [&](ml::vec3f & p) { p = translation * p; });
		std::for_each(points_b_icp.begin(), points_b_icp.end(), [&](ml::vec3f & p) { p = translation * p; });
		_registration = std::make_unique<NonRigidRegistration>(points_a_icp, points_b_icp);
		renderRegisteredPoints();
	}
	else {		
		if (_registration->solve()) {
			std::cout << "solve non rigid registration" << std::endl;
			renderRegisteredPoints();
		}
		else {
			_selected_frame_for_registration.clear();
			std::cout << "select next two frames" << std::endl;
		}
	}	
}

void ShowKinectData::renderRegisteredPoints()
{
	std::vector<ml::vec3f> render_points_a = _registration->getPointsA();
	std::vector<ml::vec3f> render_points_b = _registration->getPointsB();
	std::vector<ml::vec3f> render_points_dg = _registration->getPointsDeformationGraph();

	render_points_a.insert(render_points_a.end(), _points_a.begin(), _points_a.end());
	render_points_b.insert(render_points_b.end(), _points_b.begin(), _points_b.end());
	
	// render point clouds
	std::vector<ml::vec4f> color_frame_dg(render_points_dg.size());
	std::fill(color_frame_dg.begin(), color_frame_dg.end(), ml::RGBColor::Blue);
	m_pointCloudFrameDG.init(*_graphics, ml::meshutil::createPointCloudTemplate(ml::Shapesf::box(0.001f), render_points_dg, color_frame_dg));

	std::vector<ml::vec4f> color_frame_A(render_points_a.size());
	std::fill(color_frame_A.begin(), color_frame_A.end(), ml::RGBColor::Orange);
	m_pointCloudFrameA.init(*_graphics, ml::meshutil::createPointCloudTemplate(ml::Shapesf::box(0.001f), render_points_a, color_frame_A));

	std::vector<ml::vec4f> color_frame_B(render_points_b.size());
	std::fill(color_frame_B.begin(), color_frame_B.end(), ml::RGBColor::Green);
	m_pointCloudFrameB.init(*_graphics, ml::meshutil::createPointCloudTemplate(ml::Shapesf::box(0.001f), render_points_b, color_frame_B));
}

void ShowKinectData::key(UINT key) {
	if (key == KEY_R) {
		auto end = std::chrono::system_clock::now();
		auto elapse = std::chrono::duration_cast<std::chrono::milliseconds>(end - _start_time).count();
		if (elapse > 500) {
			_start_time = std::chrono::system_clock::now();
			_record_frames = !_record_frames;
			if (_record_frames)
				std::cout << "start recording frames" << std::endl;
			else
				std::cout << "stop recording frames" << std::endl;
		}
	}
	else if (key == KEY_2) 
	{
		_current_frame++;
		if (_current_frame == _frame)
			_current_frame = 0;
		renderPoints(_current_frame);
	}
	else if (key == KEY_1)
	{
		if (_current_frame == 0)
			_current_frame = _frame - 1;
		else
			_current_frame--;
		renderPoints(_current_frame);
	}
	else if (key == KEY_I)
	{
		if (_selected_frame_for_registration.size() < 2) {
			if (_selected_frame_for_registration.empty() || (_selected_frame_for_registration.size() == 1 && _selected_frame_for_registration[0] != _current_frame)) {
				std::cout << "select frame " << _current_frame << " for non rigid registration" << std::endl;
				_selected_frame_for_registration.push_back(_current_frame);
			}
		}
		else {
			//std::cout << "calculate icp between the two selected frames" << std::endl;
			//icp(_selected_frame_for_registration[0], _selected_frame_for_registration[1]);
			std::cout << "init non rigid registration between the two selected frames" << std::endl;
			non_rigid_registration(_selected_frame_for_registration[0], _selected_frame_for_registration[1]);
			//std::cout << "finished icp" << std::endl;
			//_selected_frame_for_registration.clear();
		}
	}
	else if (key == KEY_P) {
		std::cout << "save recorded frames as .sens file" << std::endl;
		std::string file = ".\\data\\captured_data.sens";
		std::ofstream output_file;
		output_file.open(file);
		//_sensor_data_wrapper->_sensor_data.saveToFile(file);
		//_sensor_data_wrapper->_sensor_data.savePointCloud(file, 0);// .saveToFile(file);
		//output_file << _sensor_data_wrapper->_sensor_data;
	}
	else if (key == KEY_L) {
		std::cout << "load recorded frames from .sens file" << std::endl;
		std::string file = ".\\data\\captured_data.sens";
		std::ifstream input_file;
		input_file.open(file);
		//input_file >> _sensor_data_wrapper->_sensor_data;
		//auto frame = _sensor_data_wrapper->_sensor_data.m_frames.size() - 1;
		//auto points = _sensor_data_wrapper->getPoints(frame);
		//m_pointCloud.init(*_graphics, ml::meshutil::createPointCloudTemplate(ml::Shapesf::box(0.002f), points /*_all_points*/));

	}
}