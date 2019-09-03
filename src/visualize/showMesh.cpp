#include "stdafx.h"
#include "showMesh.h"
#include "algo/registration/options/ceres_option.h"
#include "algo/registration/interface/registration.h"
#include "visualizer/image_folder_name.h"
#include "visualizer/sequence_visualizer.h"
#include "visualizer/registration_visualizer.h"
#include "parser.h"
#include <algorithm>

void ShowMesh::createRegistration()
{
	if (!_registration_visualizer) {
		auto save_images_folder = imageFolderName(_options);
		auto logger = std::make_shared<FileWriter>(save_images_folder + "/" + _options.input_mesh_sequence.output_folder_name + "_log.txt");

		if (_options.sequence_options.enable) {
			auto register_sequence_of_frames = Registration::createSequenceRegistration(_options, logger, _input_mesh);
			_registration_visualizer = std::make_shared<Visualizer::SequenceRegistrationVisualizer>(std::move(register_sequence_of_frames), _renderer, save_images_folder, logger);
		}
		else if (_selected_frame_for_registration.size() == 2) {
			auto & source = _input_mesh->getMesh(_selected_frame_for_registration[0]);
			auto & target = _input_mesh->getMesh(_selected_frame_for_registration[1]);

			auto registration = Registration::createRegistration(_options, logger, source, target);
			_registration_visualizer = std::make_shared<Visualizer::RegistrationVisualizer>(std::move(registration), _renderer, save_images_folder, logger);
		}

		_mesh_visualizer->clear();
	}
}

void ShowMesh::renderRegistration()
{
	if (_registration_visualizer)
	{
		_registration_visualizer->visualize(_render_mode);
	}
}

void ShowMesh::renderCurrentMesh()
{	
	std::vector<std::pair<unsigned int, ml::RGBColor>> frames;
	if (_selected_frame_for_registration.empty())
	{
		// current mesh	
		frames.push_back(std::make_pair(_current_frame, ml::RGBColor::White));
	}
	// selected frames
	else if (_selected_frame_for_registration.size() == 1) {
		frames.push_back(std::make_pair(_current_frame, ml::RGBColor::White));
		frames.push_back(std::make_pair(_selected_frame_for_registration[0], ml::RGBColor::Cyan));
	}
	else if (_selected_frame_for_registration.size() >= 2) {
		frames.push_back(std::make_pair(_selected_frame_for_registration[0], ml::RGBColor::Cyan));
		frames.push_back(std::make_pair(_selected_frame_for_registration[1], ml::RGBColor::Green));
	}
	
	_mesh_visualizer->visualize(frames, true);	
}


void ShowMesh::registration()
{
	if (_registration_visualizer)
	{
		if(_solve_registration && !_registration_visualizer->finished())
			_registration_visualizer->registration();
		_registration_visualizer->visualize(_render_mode);
	}
}

void ShowMesh::render(ml::Cameraf& camera)
{
	if (_registration_visualizer) {
		registration();
	}
	else {
		renderCurrentMesh();
	}

	_renderer->render(camera);

	if (_registration_visualizer) {
		_registration_visualizer->saveImage();
	}
}


void ShowMesh::key(UINT key) 
{
	if (key == KEY_2)
	{
		_current_frame++;
		if (_current_frame >= _input_mesh->size())
			_current_frame = 0;
		renderCurrentMesh();
	}
	else if (key == KEY_1)
	{
		if (_current_frame == 0)
			_current_frame = _input_mesh->size() - 1;
		else
			_current_frame--;
		renderCurrentMesh();
	}
	else if (key == KEY_I)
	{
		if (_selected_frame_for_registration.size() < 2) {
			if (_selected_frame_for_registration.empty() || 
				(_selected_frame_for_registration.size() == 1 && _selected_frame_for_registration[0] != _current_frame)) {
				std::cout << "select frame " << _current_frame << " for registration" << std::endl;
				_selected_frame_for_registration.push_back(_current_frame);
			}
		}
	}
	else if (key == KEY_O || key == KEY_V || key == KEY_B || key == KEY_N || key == KEY_M)
	{
		if (_selected_frame_for_registration.size() == 2) {
			if (!_registration_visualizer) {
				if (key == KEY_O) {
					std::cout << "init rigid registration between the two selected frames" << std::endl;
					_options.type = Registration::RegistrationType::Rigid;
				}
				else if (key == KEY_V) {
					std::cout << "init embedded deformation non rigid registration between the two selected frames" << std::endl;
					_options.type = Registration::RegistrationType::ED;
				}
				else if (key == KEY_B) {
					std::cout << "init embedded deformation non rigid registration without icp between the two selected frames" << std::endl;
					_options.type = Registration::RegistrationType::ED_WithoutICP;
				}
				else if (key == KEY_N) {
					std::cout << "init as rigid as possible non rigid registration between the two selected frames" << std::endl;
					_options.type = Registration::RegistrationType::ARAP;
				}
				else if (key == KEY_M) {
					std::cout << "init as rigid as possible non rigid registration without icpbetween the two selected frames" << std::endl;
					_options.type = Registration::RegistrationType::ARAP_WithoutICP;
				}
				createRegistration();
			}
			else {				
				_solve_registration = true;
			}
		}
	}
	else if (key == KEY_7 || key == KEY_U)
	{		
		if (!_registration_visualizer) {
			if (key == KEY_7) {
				std::cout << "register all frames with rigid registration " << std::endl;
				_options.type = Registration::RegistrationType::Rigid_AllFrames;
			}
			else if (key == KEY_U)
			{
				std::cout << "register all frames as rigid as possible " << std::endl;				
				_options.type = Registration::RegistrationType::ARAP_AllFrames;
			}
			else if (key == KEY_P)
			{
				std::cout << "register all frames embdedded deformation " << std::endl;
				_options.type = Registration::RegistrationType::ED_AllFrames;
			}
			_current_frame = 0;
			createRegistration();
			_solve_registration = true;
		}
	}
	else if (key == KEY_G)
	{
		_render_mode.show_deformation_graph = !_render_mode.show_deformation_graph;
		std::string visible = (_render_mode.show_deformation_graph) ? "show" : "hide";
		std::cout << visible << " deformation graph" << std::endl;
		renderRegistration();
	}
	else if (key == KEY_H)
	{
		if (_render_mode.mode == Visualizer::RegistrationRenderMode::ALL)
			_render_mode.mode = Visualizer::RegistrationRenderMode::DEFORMATION;
		else if (_render_mode.mode == Visualizer::RegistrationRenderMode::DEFORMATION)
			_render_mode.mode = Visualizer::RegistrationRenderMode::TARGET;
		else if (_render_mode.mode == Visualizer::RegistrationRenderMode::TARGET)
			_render_mode.mode = Visualizer::RegistrationRenderMode::ONLY_DEFORMATION_GRAPH;
		else if (_render_mode.mode == Visualizer::RegistrationRenderMode::ONLY_DEFORMATION_GRAPH)
			_render_mode.mode = Visualizer::RegistrationRenderMode::NONE;
		else
			_render_mode.mode = Visualizer::RegistrationRenderMode::ALL;
		
		std::string mode = "";
		if (_render_mode.mode == Visualizer::RegistrationRenderMode::ALL)
			mode = "ALL";
		else if (_render_mode.mode == Visualizer::RegistrationRenderMode::DEFORMATION)
			mode = "DEFORMATION";
		else if (_render_mode.mode == Visualizer::RegistrationRenderMode::TARGET)
			mode = "TARGET";
		else if (_render_mode.mode == Visualizer::RegistrationRenderMode::ONLY_DEFORMATION_GRAPH)
			mode = "ONLY_DEFORMATION_GRAPH";
		else if (_render_mode.mode == Visualizer::RegistrationRenderMode::NONE)
			mode = "NONE";
		std::cout << "render mesh mode: " << mode << std::endl;

		renderRegistration();
	}
	else if (key == KEY_J)
	{
		_render_mode.show_reference = !_render_mode.show_reference;
		std::string visible = (_render_mode.show_reference) ? "show" : "hide";
		std::cout << visible << " reference mesh" << std::endl;
		renderRegistration();
	}
	else if (key == KEY_K)
	{
		_render_mode.show_error = !_render_mode.show_error;
		std::string visible = (_render_mode.show_error) ? "enable" : "disable";
		std::cout << visible << " error evaluation" << std::endl;
		renderRegistration();
	}
}


void ShowMesh::init(ml::ApplicationData &app)
{
	_solve_registration = false;
		
	ml::mat4f scale = ml::mat4f::scale(0.01);
	ml::mat4f rotation = ml::mat4f::rotationX(-90.);
	ml::mat4f transform = ml::mat4f::translation({ -0.45f, -5.f, 1.05f });
	ml::mat4f transformation = transform * rotation * scale;

	_render_mode.mode = Visualizer::RegistrationRenderMode::ALL;

	bool test = false;

	bool load_all_frames = false;
	unsigned int number_of_frames_to_load = 10;
	if (!test) {
		//_options.input_mesh_sequence = inputById("puppet");
		//_options.input_mesh_sequence = inputById("paperbag");
		//_options.input_mesh_sequence = inputById("hand");
		_options.input_mesh_sequence = inputById("head");

		auto input_mesh = std::make_shared<MeshReader>(_options.input_mesh_sequence.file_path, _options.input_mesh_sequence.file_name, transformation, _options.input_mesh_sequence.start_index);
		if (load_all_frames) {
			input_mesh->processAllFrames();
		}
		else {
			for (unsigned int i = 0; i < number_of_frames_to_load; i++) {
				input_mesh->processFrame();
			}
		}
		_input_mesh = std::move(input_mesh);


		// options
		_options.evaluate_residuals = true;
		_options.deformation_graph.edge_length = 0.4;// 0.15;#
		_options.deformation_graph.number_of_interpolation_neighbors = 3;// min number of interpolation neighbors;

		_options.ignore_border_vertices = false;
		_options.use_vertex_random_probability = 0.5;
		_options.max_iterations = 25;
		_options.smooth = 10.;
		_options.fit = 5.;

		// refine deformation graph
		//_options.refinement.levels = 4;
		//_options.refinement.min_edge_length = 0.05;
		//_options.use_adaptive_rigidity_cost = false;
		//_renderer->_dg_edge_color = Visualize::EdgeColor::SmoothCost;

		// adaptive rigidity cost function
		_options.adaptive_rigidity.enable = true;
		_options.adaptive_rigidity.adaptive_rigidity = Registration::AdaptiveRigidity::RIGIDITY_COST;
		_options.deformation_graph.edge_length = 0.3;
		//_renderer->_dg_edge_color = Visualize::EdgeColor::RigidityValue;
		_options.use_vertex_random_probability = 0.1;
	
	}
	else {
		_input_mesh = std::make_shared<DeformationMeshFrames>();
		//auto reference_registration_mesh = std::make_shared<DeformationMeshFrames>();
		//_input_mesh = std::make_shared<HierarchicalDeformationGraphReader>(reference_registration_mesh, 4);
		_render_mode.mode = Visualizer::RegistrationRenderMode::ONLY_DEFORMATION_GRAPH;

		_options.input_mesh_sequence.output_folder_name = "test";
	}	
	_renderer = std::make_unique<Renderer>(&app.graphics);
	_mesh_visualizer = std::make_shared<Visualizer::MeshVisualizer>(_renderer, _input_mesh);
	_mesh_visualizer->visualize(std::make_pair(_current_frame, ml::RGBColor::White), true);

	std::cout << "controls:" << std::endl;
	std::cout << "    show / hide deformation graph:                 KEY_G" << std::endl;
	std::cout << "    show / hide mesh:                              KEY_H" << std::endl;
	std::cout << "    show / hide reference mesh:                    KEY_J" << std::endl;
	std::cout << "    show / hide error to reference mesh:           KEY_K" << std::endl;
	std::cout << "    enable / disable error evalutation:            KEY_L" << std::endl;
	std::cout << "    show next frame:                               KEY_2" << std::endl;
	std::cout << "    show previous frame:                           KEY_1" << std::endl;
	std::cout << "    select frame for registration:                 KEY_I" << std::endl;
	std::cout << "    rigid registration:                            KEY_O" << std::endl;
	std::cout << "    non rigid registration (ed):                   KEY_V" << std::endl;
	std::cout << "    non rigid registration (ed without icp):       KEY_B" << std::endl;
	std::cout << "    non rigid registration (arap):                 KEY_N" << std::endl;
	std::cout << "    non rigid registration (arap without icp):     KEY_M" << std::endl;
	std::cout << "    rigid registration all frames                  KEY_7" << std::endl;
	std::cout << "    non rigid registration for all frames (arap):  KEY_U" << std::endl;
	std::cout << "    non rigid registration for all frames (ed):    KEY_P" << std::endl;

}
