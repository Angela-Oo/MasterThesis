#include "showKinectData.h"
#include <numeric>
#include "algo/registration/icp.h"
#include "algo/registration/embedded_deformation.h"
#include "input_reader/kinect_reader.h"
#include <ceres/ceres.h>

using namespace ml;

void ShowKinectData::init(ml::ApplicationData &app)
{
	_start_time = std::chrono::system_clock::now();
	_reader = std::make_unique<KinectReader>();
	_point_renderer = std::make_unique<PointsRenderer>(app);
}

void ShowKinectData::renderPoints(int frame)
{
	auto points = _reader->getPoints(frame, 3);
	_point_renderer->insertPoints("frameA", points, ml::RGBColor::Orange);
}

void ShowKinectData::renderRegisteredPoints()
{
	auto render_points_a = _registration->getDeformedPoints();
	auto render_points_b = _registration->getTarget();
	//std::vector<ml::vec3f> render_points_dg = _registration->getPointsDeformationGraph();

	// render point clouds
	//_point_renderer->insertPoints("frame_deformation_graph", render_points_dg, ml::RGBColor::Blue);
	_point_renderer->insertPoints("frame_registered_A", render_points_a, ml::RGBColor::Orange);
	_point_renderer->insertPoints("frame_registered_B", render_points_b, ml::RGBColor::Green);
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

	_point_renderer->render(camera);
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

	_point_renderer->insertPoints("frameA", points_a, ml::RGBColor::Orange);
	_point_renderer->insertPoints("frameB", points_b, ml::RGBColor::Green);
}

void ShowKinectData::non_rigid_registration(int frame_a, int frame_b)
{
	if (!_registration) {
		auto points_a = _reader->getPoints(frame_a, 2);
		auto points_b = _reader->getPoints(frame_b, 2);

		_point_renderer->insertPoints("frameA", points_a, ml::RGBColor::Orange);
		_point_renderer->insertPoints("frameB", points_b, ml::RGBColor::Green);

		auto points_a_icp = points_a;
		auto points_b_icp = points_b;

		auto translation = ml::mat4f::translation({ 1.f, 0., 0. });
		std::for_each(points_a_icp.begin(), points_a_icp.end(), [&](ml::vec3f & p) { p = translation * p; });
		std::for_each(points_b_icp.begin(), points_b_icp.end(), [&](ml::vec3f & p) { p = translation * p; });

		ml::TriMeshf mesh_a;
		for (auto & p : points_a_icp)
			mesh_a.m_vertices.push_back(p);

		ml::TriMeshf mesh_b;
		for (auto & p : points_b_icp)
			mesh_b.m_vertices.push_back(p);

		_registration = std::make_unique<ED::EmbeddedDeformation>(mesh_a, mesh_b, ceresOption());
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