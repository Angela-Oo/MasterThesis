#include "showMesh.h"


void ShowMesh::renderPoints(unsigned int frame)
{
	_mesh_renderer->insertMesh("mesh", _mesh_reader->getMesh(_current_frame));
}

void ShowMesh::init(ml::ApplicationData &app)
{
	_mesh_renderer = std::make_unique<MeshRenderer>(app, PhongShader());

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
	//_mesh_reader = std::make_unique<MeshReader>("../input_data/HaoLi/head/finalRegistration/", "meshOfFrame", transformation);
	//_mesh_reader = std::make_unique<MeshReader>("../input_data/HaoLi/head/headInputScan/", "meshOfFrame", transformation);

	// hand
	_mesh_reader = std::make_unique<MeshReader>("../input_data/HaoLi/hand/hand1-registrationOutput/", "meshOfFrame", transformation);
	//_mesh_reader = std::make_unique<MeshReader>("../input_data/HaoLi/hand/hand-inputScans/", "meshOfFrame", transformation);

	//_mesh_reader->processAllFrames();
	for (int i = 0; i < 5; i++)
		_mesh_reader->processFrame();
	renderPoints(_current_frame);
}

void ShowMesh::render(ml::Cameraf& camera)
{
	_mesh_renderer->render(camera);
}


void ShowMesh::key(UINT key) {

	if (key == KEY_2)
	{
		_current_frame++;
		if (_current_frame >= _mesh_reader->frame())
			_current_frame = 0;
		renderPoints(_current_frame);
	}
	else if (key == KEY_1)
	{
		if (_current_frame == 0)
			_current_frame = _mesh_reader->frame() - 1;
		else
			_current_frame--;
		renderPoints(_current_frame);
	}
	else if (key == KEY_I)
	{
		//if (_selected_frame_for_registration.size() < 2) {
		//	if (_selected_frame_for_registration.empty() || (_selected_frame_for_registration.size() == 1 && _selected_frame_for_registration[0] != _current_frame)) {
		//		std::cout << "select frame " << _current_frame << " for non rigid registration" << std::endl;
		//		_selected_frame_for_registration.push_back(_current_frame);
		//	}
		//}
		//else if (!_registration) {
		//	std::cout << "init non rigid registration between the two selected frames" << std::endl;
		//	non_rigid_registration(_selected_frame_for_registration[0], _selected_frame_for_registration[1]);
		//}
		//else {
		//	_solve_non_rigid_registration = true;
		//}
	}
}