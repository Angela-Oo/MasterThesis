#include "showMesh.h"
#include "algo/evaluate_registration.h"
#include <algorithm>
#include <cmath>

void ShowMesh::nonRigidRegistration(int frame_a, int frame_b)
{
	if (!_registration) {
		auto& points_a = _input_mesh->getMesh(frame_a);
		auto& points_b = _input_mesh->getMesh(frame_b);
		
		_registration = std::make_unique<NonRigidRegistration>(points_a, points_b, 300);
		//_registration = std::make_unique<NonRigidRegistrationMesh>(_input_mesh->getMesh(frame_a), _input_mesh->getMesh(frame_b), 300);

		renderRegistration();
		//_render_mesh = false;
	}
	else {
		if (_registration->solve()) {
			std::cout << "solve non rigid registration" << std::endl;
			renderRegistration();

		}
		else {
			_selected_frame_for_registration.clear();
			_solve_non_rigid_registration = false;
			_registration.reset();
			std::cout << "finished, select next two frames" << std::endl;
		}
	}
}

void ShowMesh::renderError()
{
	if (_registration) {
		auto registered_points_a = _registration->getPointsA();
		auto nearest_reference_points = evaluate_error(registered_points_a, _reference_registration_mesh->getMesh(_selected_frame_for_registration[1]));

		auto distance_errors = evaluate_distance_error(registered_points_a, nearest_reference_points);
		float average = std::accumulate(distance_errors.begin(), distance_errors.end(), 0.0) / distance_errors.size();
		float max = *std::max_element(distance_errors.begin(), distance_errors.end());
		std::cout << "mean distance error: " << average << " max distance error: " << max << std::endl;

		std::vector<ml::vec3f> positions_a;
		for (auto & p : registered_points_a.getVertices())
			positions_a.push_back(p.position);
		_point_renderer->insertLine("line", positions_a, nearest_reference_points, ml::RGBColor::Red, 0.0005);
	}
}

void ShowMesh::renderRegisteredPoints()
{
	if (_registration)
	{
		auto render_points_a = _registration->getPointsA();
		auto render_points_b = _registration->getPointsB();
		std::vector<ml::vec3f> render_points_dg = _registration->getPointsDeformationGraph();

		// render point clouds
		_point_renderer->insertPoints("frame_deformation_graph", render_points_dg, ml::RGBColor::Blue, 0.004);
		_point_renderer->insertPoints("frame_registered_A", render_points_a, ml::RGBColor::Cyan);
		_point_renderer->insertPoints("frame_registered_B", render_points_b, ml::RGBColor::Green);
	}
}

void ShowMesh::renderMesh()
{
	_mesh_renderer->clear();

	if (_render_mesh) {
		if (!_registration) {
			_mesh_renderer->insertMesh("mesh", _input_mesh->getMesh(_current_frame));
		}
		if (_selected_frame_for_registration.size() >= 1)
		{
			ml::vec4f color = ml::RGBColor::Cyan.toVec4f();
			_mesh_renderer->insertMesh("mesh_a", _input_mesh->getMesh(_selected_frame_for_registration[0]), color);
		}
		if (_selected_frame_for_registration.size() >= 2)
		{
			ml::vec4f color = ml::RGBColor::Green.toVec4f();
			color.w = 0.3f;
			_mesh_renderer->insertMesh("mesh_b", _input_mesh->getMesh(_selected_frame_for_registration[1]), color);
			//_mesh_renderer->insertMesh("reference", _reference_registration_mesh->getMesh(_selected_frame_for_registration[1]));
		}
	}
	if (_render_reference_mesh) {
		_mesh_renderer->insertMesh("reference", _reference_registration_mesh->getMesh(_current_frame));
	}
}

void ShowMesh::renderRegistration()
{
	renderMesh();
	renderRegisteredPoints();
	renderError();
}

void ShowMesh::render(ml::Cameraf& camera)
{
	if (_solve_non_rigid_registration && _registration && _selected_frame_for_registration.size() == 2) {
		nonRigidRegistration(_selected_frame_for_registration[1], _selected_frame_for_registration[0]);
	}
	_mesh_renderer->render(camera);
	_point_renderer->render(camera);
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
			if (_selected_frame_for_registration.empty() || (_selected_frame_for_registration.size() == 1 && _selected_frame_for_registration[0] != _current_frame)) {
				std::cout << "select frame " << _current_frame << " for non rigid registration" << std::endl;
				_selected_frame_for_registration.push_back(_current_frame);
			}
		}
		else if (!_registration) {
			std::cout << "init non rigid registration between the two selected frames" << std::endl;
			nonRigidRegistration(_selected_frame_for_registration[0], _selected_frame_for_registration[1]);
		}
		else {
			_solve_non_rigid_registration = true;
		}
	}
	else if (key == KEY_O)
	{
		_render_mesh = true;
		std::cout << "show mesh" << std::endl;
		renderRegistration();
	}
	else if (key == KEY_P)
	{
		_render_mesh = false;
		std::cout << "hide mesh" << std::endl;
		renderRegistration();
	}
	else if (key == KEY_K)
	{
		_render_reference_mesh = true;
		std::cout << "show reference mesh" << std::endl;
		renderRegistration();
	}
	else if (key == KEY_L)
	{
		_render_reference_mesh = false;
		std::cout << "hide reference mesh" << std::endl;
		renderRegistration();
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
	//_mesh_reader = std::make_unique<MeshReader>("../input_data/HaoLi/puppet/finalRegistration/", "mesh_1",  transformation);
	//_mesh_reader = std::make_unique<MeshReader>("../input_data/HaoLi/puppet/puppetInputScans/", "meshOfFrame", transformation);	

	// paperbag
	//_mesh_reader = std::make_unique<MeshReader>("../input_data/HaoLi/paperbag/finalregistration/", "meshOfFrame", transformation);
	//_mesh_reader = std::make_unique<MeshReader>("../input_data/HaoLi/paperbag/inputscans/", "meshOfFrame", transformation);

	// head
	//_reference_registration_mesh = std::make_unique<MeshReader>("../input_data/HaoLi/head/finalRegistration/", "meshOfFrame", transformation);
	//_input_mesh = std::make_unique<MeshReader>("../input_data/HaoLi/head/headInputScans/", "meshOfFrame", transformation);

	// hand
	_reference_registration_mesh = std::make_unique<MeshReader>("../input_data/HaoLi/hand/hand1-registrationOutput/", "meshOfFrame", transformation, 1);
	_input_mesh = std::make_unique<MeshReader>("../input_data/HaoLi/hand/hand-inputScans/", "meshOfFrame", transformation, 0);

	//_mesh_reader->processAllFrames();
	for (int i = 0; i < 2; i++) {
		_input_mesh->processFrame();
		_reference_registration_mesh->processFrame();
	}
	renderMesh();
}