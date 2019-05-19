#include "stdafx.h"
#include "showMesh.h"
#include <algorithm>
#include <cmath>
#include "algo/registration/as_rigid_as_possible.h"
#include "algo/registration/embedded_deformation.h"
#include "algo/registration/rigid_registration.h"
#include "algo/registration/ceres_option.h"

void ShowMesh::nonRigidRegistration()
{

	if (!_registration && _selected_frame_for_registration.size() == 2) {
		auto frame_a = _selected_frame_for_registration[0];
		auto frame_b = _selected_frame_for_registration[1];
		auto & source = _input_mesh->getMesh(frame_a);
		auto & target = _input_mesh->getMesh(frame_b);
		auto option = ceresOption();
		if(_registration_type == RegistrationType::ED)
			_registration = std::make_unique<ED::EmbeddedDeformation>(source, target, option, _number_of_deformation_graph_nodes, _logger);
		else if(_registration_type == RegistrationType::ED_WithoutICP)
			_registration = std::make_unique<ED::EmbeddedDeformationWithoutICP>(source, target, _input_mesh->getFixedPositions(frame_b), option, _logger);
		else if(_registration_type == RegistrationType::ASAP)
			//_registration = std::make_unique<AsRigidAsPossible>(source, target, option, _number_of_deformation_graph_nodes, _logger);
			_registration = std::make_unique<AsRigidAsPossibleWithoutICP>(source, target, option, _number_of_deformation_graph_nodes, _logger);
		else if(_registration_type == RegistrationType::ASAP_WithoutICP)
			_registration = std::make_unique<AsRigidAsPossibleWithoutICP>(source, target, _input_mesh->getFixedPositions(frame_b), option, _logger);
		else
			_registration = std::make_unique<RigidRegistration>(source, target, option, _logger);
		renderRegistration();
	}
	else {
		if (!_registration->solveIteration()) {
			renderRegistration();
		}
		else {
			_selected_frame_for_registration.clear();
			_solve_registration = false;
			std::cout << std::endl << "finished, select next two frames" << std::endl;
		}
	}
}

void ShowMesh::solveAllNonRigidRegistration()
{	
	if (!_registration_frames) {
		std::vector<Mesh> meshes;
		for (int i = 0; i < _input_mesh->frame(); ++i) {
			meshes.push_back(_input_mesh->getMesh(i));
		}
		//_registration_frames = std::make_unique<NonRigidRegistrationFrames>(meshes, 300);
		_registration_frames = std::make_unique<NonRigidRegistrationAllFrames<AsRigidAsPossible, DeformationGraph<ARAPGraph, ARAPNode>>>(meshes, _number_of_deformation_graph_nodes);
		//_registration_frames = std::make_unique<NonRigidRegistrationAllFrames<ED::EmbeddedDeformation, DeformationGraph<ED::Graph, ED::Node>>>(meshes, _number_of_deformation_graph_nodes);
		renderRegistration();
	}
	else {
		if (_registration_frames->solve()) {
			renderRegistration();
		}
		else {
			std::cout << std::endl << "finished registration" << std::endl;
			_solve_registration = false;
		}
	}
}

void ShowMesh::renderError()
{
	if (_registration)
	{
		auto gradient = _registration->gradient();
		
		if (gradient.fit_point_to_plane_gradient.size() == gradient.point.size()) {
			std::vector<ml::vec3f> point_to_plane;
			std::vector<ml::vec3f> point_to_point;
			std::vector<ml::vec3f> smooth;
			for (int i = 0; i < gradient.point.size(); ++i)
			{
				point_to_plane.emplace_back(ml::vec3f(gradient.point[i] + gradient.fit_point_to_plane_gradient[i].translation));
				point_to_point.emplace_back(ml::vec3f(gradient.point[i] + gradient.fit_point_to_point_gradient[i].translation));
				smooth.emplace_back(ml::vec3f(gradient.point[i] + gradient.smooth_gradient[i].translation));
			}
			_point_renderer->insertLine("gradient", gradient.point, point_to_plane, ml::RGBColor::Red);
			_point_renderer->insertLine("gradient_point", gradient.point, point_to_point, ml::RGBColor::Orange);
			_point_renderer->insertLine("gradient_smooth", gradient.point, smooth, ml::RGBColor::Yellow);
		}
	}
	if (_registration && _calculate_error) {
		if (!_error_evaluation) {
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
			std::vector<ml::vec3f> positions_a;
			for (auto & p : registered_points_a.getVertices())
				positions_a.push_back(p.position);
			_point_renderer->insertLine("error", positions_a, nearest_reference_points, ml::RGBColor::Red, 0.0005);
		}
		else {
			_point_renderer->removePoints("error");
		}
	}
	else {
		_point_renderer->removePoints("error");
	}
}



void ShowMesh::renderRegistrationTwoFrames()
{
	if (_registration)
	{
		if (_render_points || _render_mesh != Render::NONE) {
			auto deformed_points = _registration->getDeformedPoints();
			
			// render point clouds
			if (_render_mesh == Render::NONE) {
				_point_renderer->insertPoints("frame_registered_A", deformed_points, ml::RGBColor::Cyan);
				if(!_point_renderer->keyExists("frame_registered_B"))
					_point_renderer->insertPoints("frame_registered_B", _registration->getTarget(), ml::RGBColor::Green);
			}
			// render mesh
			if (_render_mesh == Render::DEFORMATION) {
				_mesh_renderer->insertMesh("mesh_a", deformed_points, ml::RGBColor::Cyan.toVec4f());
				_mesh_renderer->removeMesh("mesh_b");
			}
			else if (_render_mesh == Render::ALL) {
				_mesh_renderer->insertMesh("mesh_a", deformed_points, ml::RGBColor::Cyan.toVec4f());
				if (!_mesh_renderer->keyExists("mesh_b"))
					_mesh_renderer->insertMesh("mesh_b", _registration->getTarget(), ml::RGBColor::Green.toVec4f());
			}
			else if(_render_mesh == Render::TARGET)	{
				if (!_mesh_renderer->keyExists("mesh_b"))
					_mesh_renderer->insertMesh("mesh_b", _registration->getTarget(), ml::RGBColor::Green.toVec4f());
				_mesh_renderer->removeMesh("mesh_a");
			}
		}

		// fixed positions
		std::vector<ml::vec3f> render_fixed_positions = _registration->getFixedPostions();
		if (!render_fixed_positions.empty())
			_point_renderer->insertPoints("frame_fixed_positions", render_fixed_positions, ml::RGBColor::Red, 0.005);

		// deformation graph
		if (_render_deformation_graph) {
			auto render_dg = _registration->getDeformationGraph();
			_point_renderer->insertLine("deformation_graph", render_dg.first, render_dg.second, ml::RGBColor::Purple);
		}
		else {
			_point_renderer->removePoints("deformation_graph");
		}
	}
}

void ShowMesh::renderRegistrationAllFrames()
{
	if (_registration_frames) {
		auto current = _registration_frames->getCurrent();		

		// mesh
		if (_render_mesh == Render::DEFORMATION || _render_mesh == Render::ALL)
		{
			if (_registration_frames->finished()) {
				for (int i = 0; i <= _registration_frames->getCurrent(); ++i) {
					_mesh_renderer->insertMesh("mesh_" + i, _registration_frames->getDeformedMesh(i), ml::RGBColor::Cyan.toVec4f());
				}
			}
			else {
				auto deformed_points = _registration_frames->getDeformedMesh(0);
				_mesh_renderer->insertMesh("mesh_" + 0, deformed_points, ml::RGBColor::Cyan.toVec4f());
			}
		}
		if (_render_mesh == Render::TARGET || _render_mesh == Render::ALL) {
			auto target_points = _registration_frames->getDeformedMesh(current);
			_mesh_renderer->insertMesh("mesh_" + current, target_points, ml::RGBColor::Green.toVec4f());
		}

		// points
		if (_render_mesh == Render::NONE) {
			auto deformed_points = _registration_frames->getDeformedMesh(0);
			auto target_points = _registration_frames->getDeformedMesh(current);
			_point_renderer->insertPoints("frame_registered_A", deformed_points, ml::RGBColor::Cyan);
			_point_renderer->insertPoints("frame_registered_B", target_points, ml::RGBColor::Green);
		}

		// target mesh 
		if (!_registration_frames->finished())
			_point_renderer->insertPoints("frame_B", _registration_frames->getMesh(_registration_frames->getCurrent()), ml::RGBColor::Yellow);
		else {
			_point_renderer->removePoints("frame_B");
		}

		// deformation graph
		if (_render_deformation_graph) {
			auto render_dg = _registration_frames->getDeformationGraph(_registration_frames->getCurrent());
			_point_renderer->insertLine("deformation_graph", render_dg.first, render_dg.second, ml::RGBColor::Purple);
		}
		else {
			_point_renderer->removePoints("deformation_graph");
		}
	}
}


void ShowMesh::renderCurrentMesh()
{
	// current mesh
	if (!_registration && !_registration_frames) {		
		_mesh_renderer->insertMesh("mesh", _input_mesh->getMesh(_current_frame));

		// render mesh if selected
		if (_selected_frame_for_registration.size() >= 1)
		{
			_mesh_renderer->insertMesh("mesh_a", _input_mesh->getMesh(_selected_frame_for_registration[0]), ml::RGBColor::Cyan.toVec4f());
			if (_selected_frame_for_registration.size() >= 2)
			{
				_mesh_renderer->insertMesh("mesh_b", _input_mesh->getMesh(_selected_frame_for_registration[1]), ml::RGBColor::Green.toVec4f());
			}
		}
	}
	else {
		_mesh_renderer->removeMesh("mesh");
	}

	// reference
	if (_render_reference_mesh) {
		_mesh_renderer->insertMesh("reference", _reference_registration_mesh->getMesh(_current_frame));
	}
	else {
		_mesh_renderer->removeMesh("reference");
	}
}

void ShowMesh::renderRegistration()
{
	renderCurrentMesh();
	renderRegistrationTwoFrames();
	renderRegistrationAllFrames();
	renderError();
}

void ShowMesh::render(ml::Cameraf& camera)
{
	_mesh_renderer->render(camera);
	_point_renderer->render(camera);

	if (_solve_registration && _registration_frames && _registration_type == RegistrationType::AllFrames) {
		solveAllNonRigidRegistration();
	}
	else if (_solve_registration && _registration && _selected_frame_for_registration.size() == 2) {
		nonRigidRegistration();
	}
	//else if (_solve_rigid_registration && _registration && _selected_frame_for_registration.size() == 2) {
	//	rigidRegistration(_selected_frame_for_registration[1], _selected_frame_for_registration[0]);
	//}
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
		if (!_registration_frames) {
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
		_render_deformation_graph = !_render_deformation_graph;
		std::string visible = (_render_deformation_graph) ? "show" : "hide";
		std::cout << visible << " deformation graph" << std::endl;
		renderRegistration();
	}
	else if (key == KEY_H)
	{
		if (_render_mesh == Render::NONE)
			_render_mesh = Render::DEFORMATION;
		else if(_render_mesh == Render::DEFORMATION)
			_render_mesh = Render::TARGET;
		else if (_render_mesh == Render::TARGET)
			_render_mesh = Render::ALL;
		else
			_render_mesh = Render::NONE;
		std::string visible = (_render_mesh != Render::NONE) ? "show" : "hide";
		std::cout << visible << " mesh" << std::endl;
		_mesh_renderer->clear();
		renderRegistration();
	}
	else if (key == KEY_J)
	{
		_render_reference_mesh = !_render_reference_mesh;
		std::string visible = (_render_reference_mesh) ? "show" : "hide";
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
	
	_mesh_renderer = std::make_unique<MeshRenderer>(app);
	_point_renderer = std::make_unique<PointsRenderer>(app);

	ml::mat4f scale = ml::mat4f::scale(0.01);
	ml::mat4f rotation = ml::mat4f::rotationX(-90.);
	ml::mat4f transform = ml::mat4f::translation({ -0.5f, 3.5f, 1.5f });
	ml::mat4f transform2 = ml::mat4f::translation({ 0.f, -10.f, 0.0f });
	ml::mat4f transformation = transform2 * transform * rotation * scale;

	// puppet
	//_reference_registration_mesh = std::make_unique<MeshReader>("../input_data/HaoLi/puppet/finalRegistration/", "mesh_1",  transformation, 1);
	//_input_mesh = std::make_unique<MeshReader>("../input_data/HaoLi/puppet/puppetInputScans/", "meshOfFrame", transformation, 0);
	//_logger = std::make_shared<FileWriter>("puppet_log.txt");

	// paperbag
	//_reference_registration_mesh = std::make_unique<MeshReader>("../input_data/HaoLi/paperbag/finalregistration/", "meshOfFrame", transformation, 1);
	//_input_mesh = std::make_unique<MeshReader>("../input_data/HaoLi/paperbag/inputscans/", "meshOfFrame", transformation, 0);

	// head
	auto reference_registration_mesh = std::make_unique<MeshReader>("../input_data/HaoLi/head/finalRegistration/", "meshOfFrame", transformation, 1);
	auto input_mesh = std::make_unique<MeshReader>("../input_data/HaoLi/head/headInputScans/", "meshOfFrame", transformation, 0);
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

	//_input_mesh = std::make_unique<DeformationMesh>();
	//_reference_registration_mesh = std::make_unique<DeformationMesh>();
	//_logger = std::make_shared<FileWriter>("test.txt");
	//_render_reference_mesh = false;
	//_calculate_error = false;
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