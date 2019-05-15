#include "showMesh.h"
#include <algorithm>
#include <cmath>



void ShowMesh::nonRigidRegistration()
{

	if (!_registration && _selected_frame_for_registration.size() == 2) {
		auto frame_a = _selected_frame_for_registration[0];
		auto frame_b = _selected_frame_for_registration[1];
		auto & source = _input_mesh->getMesh(frame_a);
		auto & target = _input_mesh->getMesh(frame_b);
		int number_of_nodes = 1000;
		auto option = ceresOption();
		if(_registration_type == RegistrationType::ED)
			_registration = std::make_unique<ED::EmbeddedDeformation>(source, target, option, number_of_nodes, _logger);
		else if(_registration_type == RegistrationType::ED_WithoutICP)
			_registration = std::make_unique<ED::EmbeddedDeformationWithoutICP>(source, target, _input_mesh->getFixedPositions(frame_b), option, _logger);
		else if(_registration_type == RegistrationType::ASAP)
			_registration = std::make_unique<AsRigidAsPossible>(source, target, option, number_of_nodes, _logger);
		else if(_registration_type == RegistrationType::ASAP_WithoutICP)
			_registration = std::make_unique<AsRigidAsPossibleWithoutICP>(source, target, _input_mesh->getFixedPositions(frame_b), option, _logger);
		else
			_registration = std::make_unique<RigidRegistration>(source, target, _logger);
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
		_registration_frames = std::make_unique<NonRigidRegistrationAllFrames<AsRigidAsPossible, DeformationGraph<ARAPGraph, ARAPNode>>>(meshes, 300);
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
			_point_renderer->insertLine("line", positions_a, nearest_reference_points, ml::RGBColor::Red, 0.0005);
		}
	}
}

void ShowMesh::renderRegisteredPoints()
{
	if (_registration)
	{
		auto render_points_a = _registration->getDeformedPoints();
		auto render_points_b = _registration->getTarget();
		std::vector<ml::vec3f> render_fixed_positions = _registration->getFixedPostions();

		// render point clouds
		_point_renderer->insertPoints("frame_registered_A", render_points_a, ml::RGBColor::Cyan);
		_point_renderer->insertPoints("frame_registered_B", render_points_b, ml::RGBColor::Green);
		_point_renderer->insertPoints("frame_fixed_positions", render_fixed_positions, ml::RGBColor::Red, 0.005);

		//_point_renderer->insertPoints("frame_deformation_graph", render_points_dg, ml::RGBColor::Blue, 0.004);
		if (_render_deformation_graph) {
			auto render_dg = _registration->getDeformationGraph();
			_point_renderer->insertLine("deformation_graph", render_dg.first, render_dg.second, ml::RGBColor::Purple);
		}
		else {
			_point_renderer->removePoints("deformation_graph");
		}
	}
	else if (_registration_frames) {
		_point_renderer->insertPoints("frame_registered_A", _registration_frames->getDeformedMesh(0), ml::RGBColor::Cyan);
		_point_renderer->insertPoints("frame_registered_B", _registration_frames->getDeformedMesh(_registration_frames->getCurrent()), ml::RGBColor::Green);
		if(!_registration_frames->finished())
			_point_renderer->insertPoints("frame_B", _registration_frames->getMesh(_registration_frames->getCurrent()), ml::RGBColor::Yellow);
		else {
			_point_renderer->removePoints("frame_B");
		}
	}
	else
	{
		_point_renderer->insertPoints("frame_B", _input_mesh->getMesh(_current_frame), ml::RGBColor::Yellow);
	}
}

void ShowMesh::renderMesh()
{
	_mesh_renderer->clear();

	if (_render_mesh) {
		if (_registration_frames) {
			for (int i = 0; i < _registration_frames->getCurrent(); ++i) {
				_mesh_renderer->insertMesh("mesh_" + i, _registration_frames->getDeformedMesh(i), ml::RGBColor::Cyan.toVec4f());
			}
			auto current = _registration_frames->getCurrent();
			_mesh_renderer->insertMesh("mesh_" + current, _registration_frames->getDeformedMesh(current), ml::RGBColor::Green.toVec4f());
		}
		else {
			if (!_registration && !_registration_frames) {
				_mesh_renderer->insertMesh("mesh", _input_mesh->getMesh(_current_frame));
			}
			if (_registration) {
				_mesh_renderer->insertMesh("mesh_a", _registration->getSource(), ml::RGBColor::Cyan.toVec4f());
				//_registration->getPointsDeformationGraph();
				_mesh_renderer->insertMesh("mesh_b", _registration->getTarget(), ml::RGBColor::Green.toVec4f());
			}
			else if (_selected_frame_for_registration.size() >= 1)
			{
				_mesh_renderer->insertMesh("mesh_a", _input_mesh->getMesh(_selected_frame_for_registration[0]), ml::RGBColor::Cyan.toVec4f());
				if (_selected_frame_for_registration.size() >= 2)
				{
					_mesh_renderer->insertMesh("mesh_b", _input_mesh->getMesh(_selected_frame_for_registration[1]), ml::RGBColor::Green.toVec4f());
				}
			}
		}
	}
	if (_render_reference_mesh) {
		_mesh_renderer->insertMesh("reference", _reference_registration_mesh->getMesh(_current_frame));
	}
}

void ShowMesh::renderRegistration()
{
	_point_renderer->clear();
	renderMesh();
	renderRegisteredPoints();
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
		renderMesh();
	}
	else if (key == KEY_1)
	{
		if (_current_frame == 0)
			_current_frame = _input_mesh->frame() - 1;
		else
			_current_frame--;
		renderMesh();
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
		_render_mesh = !_render_mesh;
		std::string visible = (_render_mesh) ? "show" : "hide";
		std::cout << visible << " mesh" << std::endl;
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
		std::string enabled = (_error_evaluation) ? "enable" : "disable";
		std::cout << enabled << " error evaluation" << std::endl;
	}
}


void ShowMesh::init(ml::ApplicationData &app)
{
	
	_mesh_renderer = std::make_unique<MeshRenderer>(app, PhongShader());
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
	renderMesh();

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