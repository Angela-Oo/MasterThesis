#pragma once
#include "../mLibInclude.h"
#include "i_showData.h"
#include "input_reader/mesh_reader.h"
#include "render/renderer.h"
#include "visualizer/i_registration_visualizer.h"
#include "visualizer/mesh_visualizer.h"
#include "algo/registration/interface/registration_options.h"

class AppRegistration : public IShowData
{
private:
	std::shared_ptr<IMeshReader> _mesh_reader;
	Registration::RegistrationOptions _options;
private:
	std::shared_ptr<Renderer> _renderer;
	std::shared_ptr<Visualizer::IRegistrationVisualizer> _registration_visualizer;
	std::shared_ptr<Visualizer::MeshVisualizer> _mesh_visualizer;	
private:
	void loadMeshReader();
	void registration();
public:
	void term();
	void render(ml::Cameraf& camera) override;	
	void key(UINT key) override;
	void init(ml::ApplicationData &app) override;
public:
	AppRegistration(const Registration::RegistrationOptions & options);
};
