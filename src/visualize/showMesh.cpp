#include "showMesh.h"

using namespace ml;

void ShowMesh::init(ml::ApplicationData &app)
{
	_mesh_renderer = std::make_unique<MeshRenderer>(app, PhongShader());

	ml::mat4f scale = ml::mat4f::scale(0.01);
	ml::mat4f rotation = ml::mat4f::rotationX(90.);
	ml::mat4f transform = ml::mat4f::translation({ -0.5f, 1.5f, 1.5f });
	ml::mat4f transformation = transform * rotation * scale;

	_mesh_reader = std::make_unique<MeshReader>("../input_data/HaoLi/hand/hand1-registrationOutput/", transformation);
	//_mesh_reader = std::make_unique<MeshReader>("../input_data/HaoLi/hand/hand-inputScans/hand-inputScans/", transformation);
}

void ShowMesh::render(ml::Cameraf& camera)
{
	_mesh_reader->processFrame();
	_mesh_renderer->insertMesh("mesh", _mesh_reader->getMesh(_mesh_reader->frame()));
	_mesh_renderer->render(camera);
}

