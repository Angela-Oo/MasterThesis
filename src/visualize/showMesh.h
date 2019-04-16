#pragma once
#include "../mLibInclude.h"
#include "i_showData.h"
#include "input_reader/i_reader.h"
#include "meshRenderer.h"
#include "input_reader/obj_reader.h"


class ShowMesh : public IShowData
{
public:
	void init(ml::ApplicationData &app) override;
	void render(ml::Cameraf& camera) override;
	void key(UINT key) override {};
private:
	std::unique_ptr<MeshRenderer> _mesh_renderer;
	std::unique_ptr<MeshReader> _mesh_reader;
};
