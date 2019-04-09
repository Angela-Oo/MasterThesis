#include "showKinectData.h"
//#include "kinect/BinaryDumpReader.h"
#include "kinect/KinectSensor.h"

#include "ext-depthcamera/sensorData.h"
#include <numeric>
#include "algo/rigid_registration/icp.h"
using namespace ml;


void ShowKinectData::renderPoints(int frame)
{
	auto points = _sensor_data_wrapper->getPoints(frame, 3);
	m_pointCloud.init(*_graphics, ml::meshutil::createPointCloudTemplate(ml::Shapesf::box(0.002f), points /*_all_points*/));
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

		_frame++;
	}

	ConstantBuffer constants;
	constants.worldViewProj = camera.getViewProj();
	constants.modelColor = ml::vec4f(1.0f, 1.0f, 1.0f, 1.0f);

	m_constants.updateAndBind(constants, 0);
	m_shaderManager.bindShaders("pointCloud");
	m_constants.bind(0);
	m_pointCloud.render();
	m_pointCloudB.render();
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

	//ICPPointSubset icp(points_a, points_b, options);
	//ml::mat4f transformation = icp.solve();

	//every 6. point Ceres Solver Iteration: 1 sub iterations: -1, Duration 2s 747ms, Total time: 4s 992ms, Initial cost: 0.185432, Final cost: 0.185105, Termination: 0
	//every 3. point Ceres Solver Iteration: 2 sub iterations: -1, Duration 12s 39ms, Total time: 33s 132ms, Initial cost: 0.406642, Final cost: 0.406586, Termination: 0
	//every 2. point Ceres Solver Iteration: 3 sub iterations: -1, Duration 29s 881ms, Total time: 113s 438ms, Initial cost: 1.38022, Final cost: 1.3802, Termination: 0
	ICPNN icpnn(points_a, points_b, options);
	ml::mat4f transformation = icpnn.solve();

	//every 6. point Ceres Solver Iteration: 0 sub iterations: -1, Duration 2s 618ms, Total time: 2s 618ms, Initial cost: 0.185864, Final cost: 0.185661, Termination: 0
	//every 3. point Ceres Solver Iteration: 0 sub iterations: -1, Duration 10s 858ms, Total time: 10s 858ms, Initial cost: 0.409008, Final cost: 0.408678, Termination: 0
	//every 2. point Ceres Solver Iteration: 0 sub iterations: -1, Duration 27s 415ms, Total time: 27s 415ms, Initial cost: 1.37843, Final cost: 1.3725, Termination: 0
	//ICP icp(points_a, points_b, options);
	//transformation = icp.solveFixNN();

	// debug
	//every 6. point Ceres Solver Iteration: 0 sub iterations: -1, Duration 10s 492ms, Total time: 10s 492ms, Initial cost: 0.25272, Final cost: 0.25272, Termination: 0
	//every 3. point  Ceres Solver Iteration: 0 sub iterations: 6, Duration 46s 824ms, Total time: 46s 824ms, Initial cost: 0.405029, Final cost: 0.405028, Termination: 0
	//every 2. point Ceres Solver Iteration: 0 sub iterations: 7, Duration 125s 500ms, Total time: 125s 500ms, Initial cost: 0.458941, Final cost: 0.458941, Termination: 0
	//ICP icp(points_a, points_b, options);
	//ml::mat4f transformation = icp.solve();

	std::for_each(points_a.begin(), points_a.end(), [&](ml::vec3f & p) { p = transformation * p; });



	std::vector<ml::vec4f> color_frame_A(points_a.size());
	std::fill(color_frame_A.begin(), color_frame_A.end(), ml::RGBColor::Orange);
	m_pointCloud.init(*_graphics, ml::meshutil::createPointCloudTemplate(ml::Shapesf::box(0.001f), points_a, color_frame_A));

	std::vector<ml::vec4f> color_frame_B(points_b.size());
	std::fill(color_frame_B.begin(), color_frame_B.end(), ml::RGBColor::Green);
	m_pointCloudB.init(*_graphics, ml::meshutil::createPointCloudTemplate(ml::Shapesf::box(0.001f), points_b, color_frame_B));
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
	else if (key == KEY_I)
	{
		std::cout << "calculate icp between last two frames" << std::endl;
		icp(_frame - 2, _frame-1);
		std::cout << "finished icp" << std::endl;
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