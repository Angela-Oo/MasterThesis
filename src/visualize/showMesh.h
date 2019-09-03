#pragma once
#include "../mLibInclude.h"
#include "i_showData.h"
#include "input_reader/i_reader.h"
#include "render/render_registration.h"
#include "input_reader/mesh_reader.h"

#include "util/file_writer.h"
#include "algo/registration/interface/registration_options.h"
#include "visualizer/mesh_visualizer.h"
#include "visualizer/i_registration_visualizer.h"
#include "render/renderer.h"

class ShowMesh : public IShowData
{
private:
	std::shared_ptr<FileWriter> _logger;
	std::shared_ptr<IMeshReader> _input_mesh;
	RegistrationOptions _options;
		
	std::shared_ptr<Renderer> _renderer;
	std::shared_ptr<Visualizer::MeshVisualizer> _mesh_visualizer;	
	std::shared_ptr<Visualizer::IRegistrationVisualizer> _registration_visualizer;

	Visualizer::Render _render_mode;
	bool _solve_registration;
	std::vector<unsigned int> _selected_frame_for_registration;
	unsigned int _current_frame;
private:
	void renderCurrentMesh();
	void renderRegistration();
	void createRegistration();
	void registration();
public:
	void init(ml::ApplicationData &app) override;
	void render(ml::Cameraf& camera) override;	
	void key(UINT key) override;
};
