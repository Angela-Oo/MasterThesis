#include "showMesh.h"

using namespace ml;

void ShowMesh::init(ml::ApplicationData &app)
{
	_mesh_renderer = std::make_unique<MeshRenderer>(app);

	_mesh_reader = std::make_unique<MeshReader>("../input_data/HaoLi/hand/hand-inputScans/hand-inputScans/");// , scale, transform);

	//MeshDataf meshData = MeshIOf::loadFromFile("scans/gates381.ply");
	MeshDataf meshData = MeshIOf::loadFromFile("../input_data/HaoLi/hand/hand-inputScans/hand-inputScans/meshOfFrame000000.obj");
	ml::TriMeshf triMesh(meshData);

	auto bounding_box = triMesh.computeBoundingBox();
	ml::mat4f center = ml::mat4f::translation(-bounding_box.getCenter());
	triMesh.transform(center);

	ml::mat4f scale = ml::mat4f::scale(0.01);
	ml::mat4f rotation = ml::mat4f::rotationX(90.);
	ml::mat4f transform = ml::mat4f::translation({ -0.5f, 1.5f, 1.5f });
	
	triMesh.transform(transform * rotation * scale);

	_mesh_renderer->insertMesh("mesh", triMesh);
}

void ShowMesh::render(ml::Cameraf& camera)
{
	_mesh_renderer->render(camera);
}

