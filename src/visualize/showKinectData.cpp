#include "showKinectData.h"
//#include "kinect/BinaryDumpReader.h"
#include "kinect/KinectSensor.h"

#include "ext-depthcamera/sensorData.h"
#include <numeric>
#include "algo/rigid_registration/icp.h"
#include "algo/non_rigid_registration/embedded_deformation.h"
#include "algo/registration.h"
#include "input_reader/kinect_reader.h"

using namespace ml;


void ShowKinectData::renderPoints(int frame)
{
	auto points = _reader->getPoints(frame, 3);
	m_pointCloudFrameA.init(*_graphics, ml::meshutil::createPointCloudTemplate(ml::Shapesf::box(0.002f), points /*_all_points*/));
}

void ShowKinectData::init(ml::ApplicationData &app)
{
	_graphics = &app.graphics;
	_start_time = std::chrono::system_clock::now();

	_reader = std::make_unique<KinectReader>();

	m_shaderManager.init(app.graphics);
	m_shaderManager.registerShader("shaders/pointCloud.hlsl", "pointCloud");
	m_constants.init(app.graphics);
}

void ShowKinectData::render(ml::Cameraf& camera)
{
	if (_record_frames) {
		_reader->processFrame();
		renderPoints(_reader->frame());
		_current_frame = _reader->frame();
	}
	else if (_solve_non_rigid_registration && _registration && _selected_frame_for_registration.size() == 2) {
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
	m_pointCloudFrameDG.render();
}


void ShowKinectData::icp(int frame_a, int frame_b)
{
	auto points_a = _reader->getPoints(frame_a);
	auto points_b = _reader->getPoints(frame_b);

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
		_points_a = _reader->getPoints(frame_a, 2);
		_points_b = _reader->getPoints(frame_b, 2);

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
			_solve_non_rigid_registration = false;
			_registration.reset();
			std::cout << "finished, select next two frames" << std::endl;
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
	m_pointCloudFrameDG.init(*_graphics, ml::meshutil::createPointCloudTemplate(ml::Shapesf::box(0.002f), render_points_dg, color_frame_dg));

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
		if (_current_frame > _reader->frame())
			_current_frame = 0;
		renderPoints(_current_frame);
	}
	else if (key == KEY_1)
	{
		if (_current_frame == 0)
			_current_frame = _reader->frame();
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
		else if(!_registration){
			std::cout << "init non rigid registration between the two selected frames" << std::endl;
			non_rigid_registration(_selected_frame_for_registration[0], _selected_frame_for_registration[1]);
		}
		else {
			_solve_non_rigid_registration = true;
		}
	}
	else if (key == KEY_P) {
		_reader->save(".\\data\\captured_data.sens");
	}
	else if (key == KEY_L) {
		_reader->load(".\\data\\captured_data.sens");
	}
}