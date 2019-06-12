#include "stdafx.h"
#include "showMesh.h"
#include <algorithm>
#include <cmath>
#include "algo/registration/rigid_registration/rigid_registration.h"
#include "algo/registration/ceres_option.h"

#include "algo/registration/embedded_deformation/ed.h"
#include "algo/registration/arap/arap.h"

//#include "algo/mesh_simplification/mesh_simplification.h"
#include "algo/surface_mesh/mesh_convertion.h"

void ShowMesh::nonRigidRegistration()
{
	if (!_registration && _selected_frame_for_registration.size() == 2) {
		auto frame_a = _selected_frame_for_registration[0];
		auto frame_b = _selected_frame_for_registration[1];
		auto & source = _input_mesh->getMesh(frame_a);
		auto & target = _input_mesh->getMesh(frame_b);
		auto option = ceresOption();
		bool evaluate_residuals = false;
		_registration = createRegistration(source, target, _registration_type, option, evaluate_residuals, _logger, _number_of_nodes, _input_mesh->getFixedPositions(frame_b));

		renderRegistration();
	}
	else {
		if (!_registration->solveIteration()) {
			renderRegistration();
		}
		else {
			//_mesh_renderer->saveCurrentWindowAsImage();
			_selected_frame_for_registration.clear();
			_solve_registration = false;
			std::cout << std::endl << "finished, select next two frames" << std::endl;
		}
	}
}

void ShowMesh::solveAllNonRigidRegistration()
{	
	if (!_register_sequence_of_frames) {
		std::vector<SurfaceMesh> meshes;
		for (int i = 0; i < _input_mesh->frame(); ++i) {
			meshes.push_back(_input_mesh->getMesh(i));
		}
		_register_sequence_of_frames = std::make_unique<SequenceRegistration>(meshes, RegistrationType::ASAP, _logger, _number_of_nodes);
		renderRegistration();
	}
	else {
		if (_register_sequence_of_frames->solve()) {
			renderRegistration();
		}
		else {
			std::cout << std::endl << "finished registration" << std::endl;
			renderRegistration();
			_solve_registration = false;
		}
	}
}

void ShowMesh::renderError()
{
	if (_registration)
	{
		//auto residuals = _registration->residuals();
		//
		////if (gradient.fit_point_to_point_gradient.size() == gradient.point.size()) {
		//for(auto & m : residuals) {
		//	
		//	//std::vector<ml::vec3f> point_to_plane;
		//	std::vector<ml::vec3f> point_to_point;
		//	//std::vector<ml::vec3f> smooth;
		//	//std::vector<ml::vec3f> all;
		//
		//	for (int i = 0; i < m.second.size(); ++i)
		//	{
		//		//point_to_plane.emplace_back(ml::vec3f(gradient.point[i] + gradient.fit_point_to_plane_gradient[i].translation));
		//		point_to_point.emplace_back(ml::vec3f(m.second..point[i] + gradient.fit_point_to_point_gradient[i].translation));
		//		//smooth.emplace_back(ml::vec3f(gradient.point[i] + gradient.smooth_gradient[i].translation));
		//		//all.emplace_back(ml::vec3f(gradient.point[i] + gradient.all[i].translation * 1000.));
		//	}
		//	//_point_renderer->insertLine("gradient", gradient.point, point_to_plane, ml::RGBColor::Red);
		//	_point_renderer->insertLine("gradient_point", gradient.point, point_to_point, ml::RGBColor::Orange);
		//	//_point_renderer->insertLine("gradient_smooth", gradient.point, smooth, ml::RGBColor::Yellow);
		//	//_point_renderer->insertLine("gradient", gradient.point, all, ml::RGBColor::Orange);
		//}
	}
	if (_registration && _calculate_error) {
		/*if (!_error_evaluation) {
			_error_evaluation = std::make_unique<ErrorEvaluation>(_reference_registration_mesh->getMesh(_selected_frame_for_registration[1]));
		}

		auto registered_points_a = _registration->getDeformedPoints();
		auto nearest_reference_points = _error_evaluation->evaluate_error(registered_points_a);

		auto distance_errors = evaluate_distance_error(registered_points_a, nearest_reference_points);
		float average = std::accumulate(distance_errors.begin(), distance_errors.end(), 0.0) / distance_errors.size();
		float max = *std::max_element(distance_errors.begin(), distance_errors.end());

		std::stringstream ss;
		ss << "ground truth error: mean: " << average << " median: " << distance_errors[distance_errors.size() / 2] << " max: " << max;
		std::cout << ss.str();
		if(_logger)
			_logger->write(ss.str());

		if (_render_error) {
			std::vector<Edge> edges;
			for (int i = 0; i < nearest_reference_points.size(); ++i) {
				Edge e;
				e.source_point = registered_points_a.m_vertices[i].position;
				e.target_point = nearest_reference_points[i];
				auto max_cost = distance_errors[distance_errors.size() / 2] * 10.;
				e.cost = std::min(max_cost, dist(e.source_point, e.target_point));
				e.cost /= max_cost;
				edges.push_back(e);
			}				
			_point_renderer->insertLine("error", edges, 0.0005);
		}
		else {
			_point_renderer->removePoints("error");
		}*/
	}
	//else {
	//	_point_renderer->removePoints("error");
	//}
}



void ShowMesh::renderCurrentMesh()
{
	// current mesh	
	bool render_current_frame = (!_registration && !_register_sequence_of_frames);
	_renderer->renderCurrentFrame(_input_mesh, _current_frame, render_current_frame);

	// selected frames
	_renderer->renderSelectedFrames(_input_mesh, _selected_frame_for_registration);

	// reference
	_renderer->renderReference(_reference_registration_mesh, _current_frame);
}

void ShowMesh::renderRegistration()
{
	renderCurrentMesh();
	_renderer->renderRegistration(_registration);
	_renderer->renderRegistrationSequence(_register_sequence_of_frames);
	renderError();
}

void ShowMesh::render(ml::Cameraf& camera)
{
	_renderer->render(camera);

	if (_solve_registration && _register_sequence_of_frames && _registration_type == RegistrationType::AllFrames) {
		_renderer->renderRegistrationSequence(_register_sequence_of_frames);
		//solveAllNonRigidRegistration();
	}
	else if (_solve_registration && _registration && _selected_frame_for_registration.size() == 2) {
		_renderer->renderRegistration(_registration);
		nonRigidRegistration();
	}
}


void ShowMesh::key(UINT key) 
{
	if (key == KEY_2)
	{
		_current_frame++;
		if (_current_frame >= _input_mesh->frame())
			_current_frame = 0;
		renderRegistration();
	}
	else if (key == KEY_1)
	{
		if (_current_frame == 0)
			_current_frame = _input_mesh->frame() - 1;
		else
			_current_frame--;
		renderRegistration();
	}
	else if (key == KEY_I)
	{
		if (_selected_frame_for_registration.size() < 2) {
			if (_selected_frame_for_registration.empty())
				_registration.reset();
			if (_selected_frame_for_registration.empty() || (_selected_frame_for_registration.size() == 1 && _selected_frame_for_registration[0] != _current_frame)) {
				std::cout << "select frame " << _current_frame << " for registration" << std::endl;
				_selected_frame_for_registration.push_back(_current_frame);
			}
		}
	}
	else if (key == KEY_O || key == KEY_V || key == KEY_B || key == KEY_N || key == KEY_M)
	{
		if (_selected_frame_for_registration.size() == 2) {
			if (!_registration) {
				if (key == KEY_O) {
					std::cout << "init rigid registration between the two selected frames" << std::endl;
					_registration_type = RegistrationType::Rigid;
				}
				else if (key == KEY_V) {
					std::cout << "init embedded deformation non rigid registration between the two selected frames" << std::endl;
					_registration_type = RegistrationType::ED;
				}
				else if (key == KEY_B) {
					std::cout << "init embedded deformation non rigid registration without icp between the two selected frames" << std::endl;
					_registration_type = RegistrationType::ED_WithoutICP;
				}
				else if (key == KEY_N) {
					std::cout << "init as rigid as possible non rigid registration between the two selected frames" << std::endl;
					_registration_type = RegistrationType::ASAP;
				}
				else if (key == KEY_M) {
					std::cout << "init as rigid as possible non rigid registration without icpbetween the two selected frames" << std::endl;
					_registration_type = RegistrationType::ASAP_WithoutICP;
				}
				nonRigidRegistration();
			}
			else {				
				_solve_registration = true;
			}
		}
	}
	else if (key == KEY_U)
	{
		std::cout << "register all frames " << std::endl;
		if (!_register_sequence_of_frames) {
			_current_frame = 0;
			_registration_type = RegistrationType::AllFrames;
			solveAllNonRigidRegistration();			
			_solve_registration = true;
		}
	}
	else if (key == KEY_T)
	{		
		if (!_registration) {
			std::cout << "test registration " << std::endl;
			_current_frame = 1;
			_selected_frame_for_registration.push_back(0);
			_selected_frame_for_registration.push_back(1);

			//_registration_type = RegistrationType::ED_WithoutICP;
			_registration_type = RegistrationType::ASAP_WithoutICP;
			nonRigidRegistration();
			_solve_registration = true;
		}
	}
	else if (key == KEY_G)
	{
		_renderer->_render_deformation_graph = !_renderer->_render_deformation_graph;
		std::string visible = (_renderer->_render_deformation_graph) ? "show" : "hide";
		std::cout << visible << " deformation graph" << std::endl;
		renderRegistration();
	}
	else if (key == KEY_H)
	{
		auto mode = _renderer->nextRenderMeshMode();
		std::string visible = (mode != Render::NONE) ? "show" : "hide";
		std::cout << visible << " mesh" << std::endl;
		//_mesh_renderer->clear();
		renderRegistration();
	}
	else if (key == KEY_J)
	{
		_renderer->_render_reference_mesh = !_renderer->_render_reference_mesh;
		std::string visible = (_renderer->_render_reference_mesh) ? "show" : "hide";
		std::cout << visible << " reference mesh" << std::endl;
		renderRegistration();
	}
	else if (key == KEY_K)
	{
		_render_error = !_render_error;
		std::string visible = (_render_error) ? "show" : "hide";
		std::cout << visible << " error to reference mesh" << std::endl;
		renderRegistration();
	}
	else if (key == KEY_L)
	{
		_calculate_error = !_calculate_error;
		std::string enabled = (_calculate_error) ? "enable" : "disable";
		std::cout << enabled << " error evaluation" << std::endl;
	}
}


void ShowMesh::init(ml::ApplicationData &app)
{
	//_number_of_nodes = 1500;
	_number_of_nodes = 5000;
	_current_frame = 0;
	_solve_registration = false;
	_registration_type = RegistrationType::ASAP;
	_render_error = false;
	_calculate_error = false;
	_renderer = std::make_unique<RenderRegistration>(&app.graphics);

	ml::mat4f scale = ml::mat4f::scale(0.01);
	ml::mat4f rotation = ml::mat4f::rotationX(-90.);
	ml::mat4f transform = ml::mat4f::translation({ -0.5f, 3.5f, 1.5f });
	ml::mat4f transform2 = ml::mat4f::translation({ 0.f, -10.f, 0.0f });
	ml::mat4f transformation = transform2 * transform * rotation * scale;

	bool test = false;
	if (!test) {
		// puppet
		//_reference_registration_mesh = std::make_unique<MeshReader>("../input_data/HaoLi/puppet/finalRegistration/", "mesh_1",  transformation, 1);
		//_input_mesh = std::make_unique<MeshReader>("../input_data/HaoLi/puppet/puppetInputScans/", "meshOfFrame", transformation, 0);
		//_logger = std::make_shared<FileWriter>("puppet_log.txt");

		// paperbag
		//_reference_registration_mesh = std::make_unique<MeshReader>("../input_data/HaoLi/paperbag/finalregistration/", "meshOfFrame", transformation, 1);
		//_input_mesh = std::make_unique<MeshReader>("../input_data/HaoLi/paperbag/inputscans/", "meshOfFrame", transformation, 0);

		// head
		auto reference_registration_mesh = std::make_shared<MeshReader>("../input_data/HaoLi/head/finalRegistration/", "meshOfFrame", transformation, 1);
		auto input_mesh = std::make_shared<MeshReader>("../input_data/HaoLi/head/headInputScans/", "meshOfFrame", transformation, 0);
		_logger = std::make_shared<FileWriter>("head_log.txt");
	
		// hand
		//auto reference_registration_mesh = std::make_unique<MeshReader>("../input_data/HaoLi/hand/hand1-registrationOutput/", "meshOfFrame", transformation, 1);
		//auto input_mesh = std::make_unique<MeshReader>("../input_data/HaoLi/hand/hand-inputScans/", "meshOfFrame", transformation, 0);
		//_logger = std::make_shared<FileWriter>("hand_log.txt");	

		////_mesh_reader->processAllFrames();
		for (int i = 0; i < 10; i++) {
			input_mesh->processFrame();
			reference_registration_mesh->processFrame();
		}
		_input_mesh = std::move(input_mesh);
		_reference_registration_mesh = std::move(reference_registration_mesh);
	}
	else {
		//_input_mesh = std::make_unique<DeformationMesh>();
		//_reference_registration_mesh = std::make_unique<DeformationMesh>();

		_input_mesh = std::make_shared<DeformationMeshFrames>();
		_reference_registration_mesh = std::make_shared<DeformationMeshFrames>();

		_logger = std::make_shared<FileWriter>("test.txt");
		_renderer->_render_reference_mesh = false;
		_calculate_error = false;
		_renderer->_render_deformation_graph = true;
	}
	renderRegistration();


	std::cout << "controls:" << std::endl;
	std::cout << "    show / hide deformation graph:             KEY_G" << std::endl;
	std::cout << "    show / hide mesh:                          KEY_H" << std::endl;
	std::cout << "    show / hide reference mesh:                KEY_J" << std::endl;
	std::cout << "    show / hide error to reference mesh:       KEY_K" << std::endl;
	std::cout << "    enable / disable error evalutation:        KEY_L" << std::endl;
	std::cout << "    show next frame:                           KEY_2" << std::endl;
	std::cout << "    show previous frame:                       KEY_1" << std::endl;
	std::cout << "    select frame for registration:             KEY_I" << std::endl;
	std::cout << "    rigid registration:                        KEY_O" << std::endl;
	std::cout << "    non rigid registration (ed):               KEY_V" << std::endl;
	std::cout << "    non rigid registration (ed without icp):   KEY_B" << std::endl;
	std::cout << "    non rigid registration (arap):             KEY_N" << std::endl;
	std::cout << "    non rigid registration (arap without icp): KEY_M" << std::endl;
	std::cout << "    non rigid registration for all frames:     KEY_U" << std::endl;

}
