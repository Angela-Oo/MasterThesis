#include "showMesh.h"

using namespace ml;

void ShowMesh::init(ml::ApplicationData &app)
{
	_mesh_renderer = std::make_unique<MeshRenderer>(app);

	MeshDataf meshData = MeshIOf::loadFromFile("scans/gates381.ply");
	ml::TriMeshf triMesh(meshData);
	_mesh_renderer->insertMesh("mesh", triMesh);
}

void ShowMesh::render(ml::Cameraf& camera)
{
	_mesh_renderer->render(camera);
}

