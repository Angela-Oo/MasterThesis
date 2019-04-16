#pragma once
#include "../mLibInclude.h"
#include "i_showData.h"
#include "input_reader/i_reader.h"
#include "meshRenderer.h"
#include "input_reader/obj_reader.h"


class ShowMesh : public IShowData
{
private:
	std::unique_ptr<MeshRenderer> _mesh_renderer;
	std::unique_ptr<MeshReader> _mesh_reader;
	unsigned int _current_frame = 0;
private:
	void renderPoints(unsigned int frame);
public:
	void init(ml::ApplicationData &app) override;
	void render(ml::Cameraf& camera) override;
	void key(UINT key) override;

};
