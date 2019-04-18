#include "showMesh.h"


void ShowMesh::nonRigidRegistration(int frame_a, int frame_b)
{
	if (!_registration) {
		auto points_a = _input_mesh->getMesh(frame_a).getVertices();
		auto points_b = _input_mesh->getMesh(frame_b).getVertices();

		std::vector<ml::vec3f> positions_a;
		for (auto p : points_a)
			positions_a.push_back(p.position);
		
		std::vector<ml::vec3f> positions_b;
		for (auto p : points_b)
			positions_b.push_back(p.position);
		
		_registration = std::make_unique<NonRigidRegistration>(positions_a, positions_b, 300);
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

void ShowMesh::renderRegisteredPoints()
{
	std::vector<ml::vec3f> render_points_a = _registration->getPointsA();
	std::vector<ml::vec3f> render_points_b = _registration->getPointsB();
	std::vector<ml::vec3f> render_points_dg = _registration->getPointsDeformationGraph();

	// render point clouds
	_point_renderer->insertPoints("frame_deformation_graph", render_points_dg, ml::RGBColor::Blue, 0.004);
	_point_renderer->insertPoints("frame_registered_A", render_points_a, ml::RGBColor::Orange);
	_point_renderer->insertPoints("frame_registered_B", render_points_b, ml::RGBColor::Green);
}

void ShowMesh::renderMesh(unsigned int frame)
{
	_mesh_renderer->clear();

	if (!_solve_non_rigid_registration) {
		_mesh_renderer->insertMesh("mesh", _input_mesh->getMesh(frame));
	}
	for (auto f : _selected_frame_for_registration)
	{
		_mesh_renderer->insertMesh("mesh" + std::to_string(f), _input_mesh->getMesh(f));
	}	
}

void ShowMesh::init(ml::ApplicationData &app)
{
	_mesh_renderer = std::make_unique<MeshRenderer>(app, PhongShader({ 1., 1., 1., 0.5 }));
	_point_renderer = std::make_unique<PointsRenderer>(app);

	ml::mat4f scale = ml::mat4f::scale(0.01);
	ml::mat4f rotation = ml::mat4f::rotationX(-90.);
	ml::mat4f transform = ml::mat4f::translation({ -0.5f, 3.5f, 1.5f });
	ml::mat4f transformation = transform * rotation * scale;

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
	_reference_registration_mesh = std::make_unique<MeshReader>("../input_data/HaoLi/hand/hand1-registrationOutput/", "meshOfFrame", transformation);
	_input_mesh = std::make_unique<MeshReader>("../input_data/HaoLi/hand/hand-inputScans/", "meshOfFrame", transformation);

	//_mesh_reader->processAllFrames();
	for (int i = 0; i < 10; i++)
		_input_mesh->processFrame();
	renderMesh(_current_frame);
}

void ShowMesh::render(ml::Cameraf& camera)
{
	if (_solve_non_rigid_registration && _registration && _selected_frame_for_registration.size() == 2) {
		nonRigidRegistration(_selected_frame_for_registration[0], _selected_frame_for_registration[1]);
	}
	_mesh_renderer->render(camera);
	_point_renderer->render(camera);
}


void ShowMesh::key(UINT key) {

	if (key == KEY_2)
	{
		_current_frame++;
		if (_current_frame >= _input_mesh->frame())
			_current_frame = 0;
		renderMesh(_current_frame);
	}
	else if (key == KEY_1)
	{
		if (_current_frame == 0)
			_current_frame = _input_mesh->frame() - 1;
		else
			_current_frame--;
		renderMesh(_current_frame);
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
}