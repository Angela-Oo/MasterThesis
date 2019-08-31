#include "stdafx.h"
#include "appRegistration.h"
#include "algo/registration/interface/registration.h"
#include "algo/surface_mesh/mesh_convertion.h"
#include "input_reader/hierarchical_deformation_graph_reader.h"
#include "image_folder_name.h"
#include <algorithm>
#include <cmath>



void AppRegistration::renderError()
{
	if (_registration && _registration->finished()) {
		if (!_error_evaluation) {
			_error_evaluation = std::make_unique<ErrorEvaluation>(_registration->getTarget());
		}
		auto registered_points_a = _registration->getDeformedPoints();
		auto nearest_reference_points = _error_evaluation->evaluate_error(registered_points_a);

		auto distance_errors = evaluate_distance_error(nearest_reference_points);

		float average = std::accumulate(distance_errors.begin(), distance_errors.end(), 0.0) / distance_errors.size();
		float max = *std::max_element(distance_errors.begin(), distance_errors.end());
		std::stringstream ss;
		ss << "error: mean " << average << ", median " << distance_errors[distance_errors.size() / 2] << ", max " << max << std::endl;
		std::cout << ss.str();
		if (_logger)
			_logger->write(ss.str());

		_renderer->renderError(nearest_reference_points);
	}
}

void AppRegistration::renderCurrentMesh()
{
	// current mesh	
	bool render_current_frame = (!_registration && !_register_sequence_of_frames);
	_renderer->renderCurrentFrame(_mesh_reader, render_current_frame);
}

void AppRegistration::renderRegistration()
{
	renderCurrentMesh();
	_renderer->renderRegistration(_registration);
	_renderer->renderRegistrationSequence(_register_sequence_of_frames);
	renderError();
}


void AppRegistration::initRegistration()
{
	_save_images_folder = imageFolderName(_options);
	_logger = std::make_shared<FileWriter>(_save_images_folder + "/" + _options.input_mesh_sequence.output_folder_name + "_log.txt");

	if (_options.sequence_options.enable) {
		_register_sequence_of_frames = createSequenceRegistration(_options.type, _options, _options.ceres_options, _logger, _mesh_reader);
	}
	else {
		auto & source = _mesh_reader->getMesh(0);
		auto & target = _mesh_reader->getMesh(_mesh_reader->size() - 1);
		
		_registration = createRegistration(_options.type,
										   _options,
										   _options.ceres_options,
										   _logger,
										   source,
										   target);		
		_image_name = "frame_" + std::to_string(_registration->currentIteration());
	}
	renderRegistration();
}



void AppRegistration::nonRigidRegistration()
{
	if (_registration) {
		bool finished = _registration->finished();
		if (!finished) {
			_registration->solveIteration();
			_image_name = "frame_" + std::to_string(_registration->currentIteration());
		}
		else {
			std::cout << std::endl << "finished, select next two frames" << std::endl;
			_image_name = "frame_finished";
			_finished = true;
		}
	}
}

void AppRegistration::solveAllNonRigidRegistration()
{
	if (_register_sequence_of_frames)
	{
		bool finished = _register_sequence_of_frames->finished();
		if (!finished) {
			bool iteration_finished = _register_sequence_of_frames->solve();
			auto save_image = _register_sequence_of_frames->saveCurrentFrameAsImage();
			if (save_image.first)
				_image_name = "frame_" + save_image.second;
		}
		else {
			std::cout << std::endl << "finished registration" << std::endl;
			_image_name = "frame_finished";
			_renderer->_render_mesh = Render::NONE;
			_finished = true;
		}
	}
}

void AppRegistration::registration()
{
	if (!_registration && !_register_sequence_of_frames) {
		initRegistration();
	}
	if (_register_sequence_of_frames) {
		solveAllNonRigidRegistration();
		_renderer->_current_frame = _register_sequence_of_frames->getCurrent();
	}
	else if (_registration) {
		nonRigidRegistration();
	}
}


void AppRegistration::render(ml::Cameraf& camera)
{
	if (!_finished) {
		registration();
		renderRegistration();
	}
	_renderer->render(camera);
	if (_image_name != "") {
		_renderer->saveCurrentWindowAsImage(_save_images_folder, _image_name);
		_image_name = "";
	}
}

void AppRegistration::key(UINT key)
{
}

void AppRegistration::initMeshReader()
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
	_renderer = std::make_unique<RenderRegistration>(&app.graphics);
	_renderer->_render_error = true;
	_renderer->_dg_edge_color = Visualize::EdgeColor::RigidityValue;

	initMeshReader();
	renderRegistration();
}


AppRegistration::AppRegistration(const RegistrationOptions & options)
	: _options(options)
{}