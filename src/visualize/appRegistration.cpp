#include "stdafx.h"
#include "appRegistration.h"
#include "visualizer/create_registration_visualizer.h"


void AppRegistration::registration()
{
	Visualizer::Render mode;
	mode.mode = _options.input_mesh_sequence.render_mode;
	if (!_registration_visualizer)
	{
		_registration_visualizer = Visualizer::createRegistrationVisualizer(_renderer, _mesh_reader, _options);
		_mesh_visualizer->clear();		
		_registration_visualizer->visualize(mode);
	}
	else if (!_registration_visualizer->finished())
	{
		_registration_visualizer->registration();
		_registration_visualizer->visualize(mode);
	}
}

void AppRegistration::render(ml::Cameraf& camera)
{
	registration();
	_renderer->render(camera);

	if (_registration_visualizer) {
		_registration_visualizer->saveImage();
		if(_registration_visualizer->finished() && _options.input_mesh_sequence.term)
		{
			term();
		}
	}
}

void AppRegistration::key(UINT key)
{
}

void AppRegistration::loadMeshReader()
{
	ml::mat4f scale = ml::mat4f::scale(0.01);
	ml::mat4f rotation = ml::mat4f::rotationX(-90.);
	ml::mat4f transform = ml::mat4f::translation({ -0.45f, -5.f, 1.05f });
	ml::mat4f transformation = transform * rotation * scale;
	auto mesh_reader = std::make_unique<MeshReader>(_options.input_mesh_sequence.file_path,
													_options.input_mesh_sequence.file_name,
													transformation,
													_options.input_mesh_sequence.start_index);

	if (_options.input_mesh_sequence.number_of_frames_to_load == -1) {
		mesh_reader->processAllFrames();
	}
	else {
		for (unsigned int i = 0; i < _options.input_mesh_sequence.number_of_frames_to_load; i++) {
			mesh_reader->processFrame();
		}
	}
	_mesh_reader = std::move(mesh_reader);
}

void AppRegistration::init(ml::ApplicationData &app)
{
	_renderer = std::make_unique<Renderer>(&app.graphics);

	loadMeshReader();

	_mesh_visualizer = std::make_shared<Visualizer::MeshVisualizer>(_renderer, _mesh_reader);
	_mesh_visualizer->visualize(std::make_pair(0, ml::RGBColor::White), true);
}

void AppRegistration::term()
{
	ExitProcess(0);
}

AppRegistration::AppRegistration(const Registration::RegistrationOptions & options)
	: _options(options)
{}