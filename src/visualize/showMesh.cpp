#include "stdafx.h"
#include "showMesh.h"
#include <algorithm>
#include <cmath>

#include "algo/registration/rigid_registration/rigid_registration.h"
#include "algo/registration/ceres_option.h"

#include "algo/registration/embedded_deformation/ed.h"
#include "algo/registration/arap/arap.h"
#include "algo/registration/arap/arap_factory.h"
#include "algo/registration/rigid_registration/rigid_factory.h"

#include "algo/surface_mesh/mesh_convertion.h"

#include <chrono>  // chrono::system_clock
#include <ctime>   // localtime
#include <iomanip> // put_time

std::string ShowMesh::getImageFolderName(RegistrationType type)
{
	auto in_time_t = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
	std::stringstream ss;
	ss << std::put_time(std::localtime(&in_time_t), "%Y-%m-%d_%H-%M");
	std::string folder_name = "registration_";
	if (type == RegistrationType::ARAP) {
		folder_name = "ARAP_";
	}
	else if (type == RegistrationType::ED) {
		folder_name = "ED_";
	}
	else if (type == RegistrationType::Rigid) {
		folder_name = "Rigid_";
	}
	else if (type == RegistrationType::ARAP_AllFrames) {
		folder_name = "ARAP_AllFrames_";
	}
	else if (type == RegistrationType::ED_AllFrames) {
		folder_name = "ED_AllFrames_";
	}
	return "images/" + _data_name + "/" + folder_name + ss.str();
}


void ShowMesh::nonRigidRegistration()
{
	if (!_registration && _selected_frame_for_registration.size() == 2) {
		auto frame_a = _selected_frame_for_registration[0];
		auto frame_b = _selected_frame_for_registration[1];
		auto & source = _input_mesh->getMesh(frame_a);
		auto & target = _input_mesh->getMesh(frame_b);
		auto option = ceresOption();

		_save_images_folder = getImageFolderName(_registration_type);
		_logger = std::make_shared<FileWriter>(_save_images_folder + "/" + _data_name + "_log.txt");

		RegistrationFactory factory;
		factory.setRegistrationType(_registration_type);
		factory.setCeresOption(option);
		factory.setRegistrationOption(_registration_options);
		factory.setLogger(_logger);	

		if (_registration_type == RegistrationType::ARAP_WithoutICP || _registration_type == RegistrationType::ED_WithoutICP) { // todo
			_registration_options.smooth = 10.;
			_registration_options.fit = 100.;
			factory.setFixedPositions(_input_mesh->getFixedPositions(frame_b));
		}

		factory.logConfiguration();
		_registration = factory.buildRegistration(source, target);
		renderRegistration();
		_image_name = "frame_" + std::to_string(_registration->currentIteration());
	}
	else {
		bool finished = _registration->finished();
		if (!finished) {
			_registration->solveIteration();
			_image_name = "frame_" + std::to_string(_registration->currentIteration());
		}
		else {
			_selected_frame_for_registration.clear();
			_solve_registration = false;
			std::cout << std::endl << "finished, select next two frames" << std::endl;
			_image_name = "frame_finished";
		}		
	}
}


void ShowMesh::solveAllNonRigidRegistration()
{	
	if (!_register_sequence_of_frames) {
		_save_images_folder = getImageFolderName(_registration_type);
		_logger = std::make_shared<FileWriter>(_save_images_folder + "/" + _data_name + "_log.txt");

		if (_registration_type == RegistrationType::ARAP_AllFrames) {
			ARAP::ARAPFactory factory(_registration_options, ceresOption(), _logger);
			_register_sequence_of_frames = std::make_unique<SequenceRegistrationT<ARAP::AsRigidAsPossible, ARAP::ARAPFactory>>(_input_mesh, factory, _logger);
		}
		else if (_registration_type == RegistrationType::Rigid_AllFrames) {
			Registration::Rigid::RigidFactory factory(_registration_options, ceresOption(), _logger);
			_register_sequence_of_frames = std::make_unique<SequenceRegistrationT<RigidRegistration, Registration::Rigid::RigidFactory>>(_input_mesh, factory, _logger);
		}
		//RegistrationType type = _registration_type == RegistrationType::ARAP_AllFrames ? RegistrationType::ARAP : RegistrationType::ED;
		//_register_sequence_of_frames = std::make_unique<SequenceRegistration>(_input_mesh, type, _logger, _registration_options);
	}
	else {
		bool finished = _register_sequence_of_frames->finished();
		if (!finished) {
			bool iteration_finished = _register_sequence_of_frames->solve();
			auto save_image = _register_sequence_of_frames->saveCurrentFrameAsImage();
			if (save_image.first)
				_image_name = "frame_" + save_image.second;// std::to_string(_register_sequence_of_frames->getCurrent());
		}
		else {		
			std::cout << std::endl << "finished registration" << std::endl;
			_solve_registration = false;
			_image_name = "frame_finished";
			_renderer->_render_mesh = Render::NONE;
		}
	}
}

void ShowMesh::renderError()
{	
	if (_registration && _calculate_error) {
		if (!_error_evaluation) {
			if(_reference_registration_mesh->size() > _selected_frame_for_registration[1])
				_error_evaluation = std::make_unique<ErrorEvaluation>(_reference_registration_mesh->getMesh(_selected_frame_for_registration[1]));
		}

		auto registered_points_a = _registration->getDeformedPoints();
		auto nearest_reference_points = _error_evaluation->evaluate_error(registered_points_a);

		auto distance_errors = evaluate_distance_error(nearest_reference_points);
		/*float average = std::accumulate(distance_errors.begin(), distance_errors.end(), 0.0) / distance_errors.size();
		float max = *std::max_element(distance_errors.begin(), distance_errors.end());

		std::stringstream ss;
		ss << "ground truth error: mean: " << average << " median: " << distance_errors[distance_errors.size() / 2] << " max: " << max;
		std::cout << ss.str();
		if(_logger)
			_logger->write(ss.str());
		*/
		_renderer->renderError(nearest_reference_points);
	}
}



void ShowMesh::renderCurrentMesh()
{
	// current mesh	
	bool render_current_frame = (!_registration && !_register_sequence_of_frames);
	_renderer->renderCurrentFrame(_input_mesh, render_current_frame);

	// selected frames
	auto selected_frames = render_current_frame ? _selected_frame_for_registration : std::vector<unsigned int>();
	_renderer->renderSelectedFrames(_input_mesh, selected_frames);	

	// reference
	_renderer->renderReference(_reference_registration_mesh);
}

void ShowMesh::renderRegistration()
{
	renderCurrentMesh();
	_renderer->renderRegistration(_registration);
	_renderer->renderRegistrationSequence(_register_sequence_of_frames);
	renderError();
}

void ShowMesh::registration()
{
	if (_register_sequence_of_frames) {
		solveAllNonRigidRegistration();
		_renderer->_current_frame = _register_sequence_of_frames->getCurrent();
	}
	else if (_registration && _selected_frame_for_registration.size() == 2) {
		nonRigidRegistration();
	}
}


void ShowMesh::render(ml::Cameraf& camera)
{
	if (_solve_registration) {
		registration();
		renderRegistration();
	}
	_renderer->render(camera);
	if (_image_name != "") {
		_renderer->saveCurrentWindowAsImage(_save_images_folder, _image_name);
		_image_name = "";
	}
}


void ShowMesh::key(UINT key) 
{
	if (key == KEY_2)
	{
		_renderer->_current_frame++;
		if (_renderer->_current_frame >= _input_mesh->size())
			_renderer->_current_frame = 0;
		renderRegistration();
	}
	else if (key == KEY_1)
	{
		if (_renderer->_current_frame == 0)
			_renderer->_current_frame = _input_mesh->size() - 1;
		else
			_renderer->_current_frame--;
		renderRegistration();
	}
	else if (key == KEY_I)
	{
		if (_selected_frame_for_registration.size() < 2) {
			if (_selected_frame_for_registration.empty())
				_registration.reset();
			if (_selected_frame_for_registration.empty() || (_selected_frame_for_registration.size() == 1 && _selected_frame_for_registration[0] != _renderer->_current_frame)) {
				std::cout << "select frame " << _renderer->_current_frame << " for registration" << std::endl;
				_selected_frame_for_registration.push_back(_renderer->_current_frame);
			}
		}
	}
	else if (key == KEY_O || key == KEY_V || key == KEY_B || key == KEY_N || key == KEY_M)
	{
		if (_selected_frame_for_registration.size() == 2) {
			if (!_registration) {
				if (key == KEY_O) {
					std::cout << "init rigid registration between the two selected frames" << std::endl;
					_registration_type = RegistrationType::Rigid;
				}
				else if (key == KEY_V) {
					std::cout << "init embedded deformation non rigid registration between the two selected frames" << std::endl;
					_registration_type = RegistrationType::ED;
				}
				else if (key == KEY_B) {
					std::cout << "init embedded deformation non rigid registration without icp between the two selected frames" << std::endl;
					_registration_type = RegistrationType::ED_WithoutICP;
				}
				else if (key == KEY_N) {
					std::cout << "init as rigid as possible non rigid registration between the two selected frames" << std::endl;
					_registration_type = RegistrationType::ARAP;
				}
				else if (key == KEY_M) {
					std::cout << "init as rigid as possible non rigid registration without icpbetween the two selected frames" << std::endl;
					_registration_type = RegistrationType::ARAP_WithoutICP;
				}
				nonRigidRegistration();
			}
			else {				
				_solve_registration = true;
			}
		}
	}
	else if (key == KEY_7)
	{		
		if (!_register_sequence_of_frames) {
			std::cout << "register all frames with rigid registration " << std::endl;
			_renderer->_current_frame = 0;
			_registration_type = RegistrationType::Rigid_AllFrames;
			solveAllNonRigidRegistration();
			_solve_registration = true;
		}
	}
	else if (key == KEY_U)
	{		
		if (!_register_sequence_of_frames) {
			std::cout << "register all frames as rigid as possible " << std::endl;
			_renderer->_current_frame = 0;
			_registration_type = RegistrationType::ARAP_AllFrames;
			solveAllNonRigidRegistration();			
			_solve_registration = true;
		}
	}
	else if (key == KEY_P)
	{	
		if (!_register_sequence_of_frames) {
			std::cout << "register all frames embdedded deformation " << std::endl;
			_renderer->_current_frame = 0;
			_registration_type = RegistrationType::ED_AllFrames;
			solveAllNonRigidRegistration();
			_solve_registration = true;
		}
	}
	else if (key == KEY_T)
	{		
		if (!_registration) {
			std::cout << "test registration " << std::endl;
			_renderer->_current_frame = 1;
			_selected_frame_for_registration.push_back(0);
			_selected_frame_for_registration.push_back(1);

			//_registration_type = RegistrationType::ED_WithoutICP;
			_registration_type = RegistrationType::ARAP_WithoutICP;
			nonRigidRegistration();
			_solve_registration = true;
		}
	}
	else if (key == KEY_G)
	{
		_renderer->_render_deformation_graph = !_renderer->_render_deformation_graph;
		std::string visible = (_renderer->_render_deformation_graph) ? "show" : "hide";
		std::cout << visible << " deformation graph" << std::endl;
		renderRegistration();
	}
	else if (key == KEY_H)
	{
		auto mode = _renderer->nextRenderMeshMode();
		std::cout << "render mesh mode: " << renderMeshModeToString(_renderer->_render_mesh) << std::endl;
		//_mesh_renderer->clear();
		renderRegistration();
	}
	else if (key == KEY_J)
	{
		_renderer->_render_reference_mesh = !_renderer->_render_reference_mesh;
		std::string visible = (_renderer->_render_reference_mesh) ? "show" : "hide";
		std::cout << visible << " reference mesh" << std::endl;
		renderRegistration();
	}
	else if (key == KEY_K)
	{
		_renderer->_render_error = !_renderer->_render_error;
		std::string visible = (_renderer->_render_error) ? "show" : "hide";
		std::cout << visible << " error to reference mesh" << std::endl;
		renderRegistration();
	}
	else if (key == KEY_L)
	{
		_calculate_error = !_calculate_error;
		std::string enabled = (_calculate_error) ? "enable" : "disable";
		std::cout << enabled << " error evaluation" << std::endl;
	}
}


void ShowMesh::init(ml::ApplicationData &app)
{
	_solve_registration = false;
	_registration_type = RegistrationType::ARAP;
	_calculate_error = false;
	_renderer = std::make_unique<RenderRegistration>(&app.graphics);
	
	ml::mat4f scale = ml::mat4f::scale(0.01);
	ml::mat4f rotation = ml::mat4f::rotationX(-90.);
	ml::mat4f transform = ml::mat4f::translation({ -0.5f, 3.5f, 1.5f });
	ml::mat4f transform2 = ml::mat4f::translation({ 0.f, -10.f, 0.0f });
	ml::mat4f transformation = transform2 * transform * rotation * scale;

	bool test = false;
	bool register_on_reference_mesh = true;
	bool load_compare_mesh = false;
	bool load_all_frames = false;
	unsigned int number_of_frames_to_load = 10;
	if (!test) {
		_registration_options.evaluate_residuals = true;
		_registration_options.dg_options.edge_length = 0.3;// 0.15;
		_registration_options.ignore_deformation_graph_border_vertices = false;
		_registration_options.dg_options.number_of_interpolation_neighbors = 4;
		_registration_options.use_vertex_random_probability = 0.5;
		_registration_options.max_iterations = 25;
		_registration_options.smooth = 10.;
		_registration_options.fit = 10.;
		// puppet
		auto reference_registration_mesh = std::make_unique<MeshReader>("../input_data/HaoLi/puppet/finalRegistration/", "mesh_1",  transformation, 0);
		auto input_mesh = std::make_unique<MeshReader>("../input_data/HaoLi/puppet/puppetInputScans/", "meshOfFrame", transformation, 0);
		_registration_options.dg_options.edge_length = 0.2;
		_registration_options.use_vertex_random_probability = 0.5;
		_data_name = "puppet";

		// paperbag
		//auto reference_registration_mesh = std::make_unique<MeshReader>("../input_data/HaoLi/paperbag/finalregistration/", "meshOfFrame", transformation, 1);
		//auto input_mesh = std::make_unique<MeshReader>("../input_data/HaoLi/paperbag/inputscans/", "meshOfFrame", transformation, 1);
		//_registration_options.dg_options.edge_length = 0.2;
		//_data_name = "paperbag";

		// head
		//auto reference_registration_mesh = std::make_shared<MeshReader>("../input_data/HaoLi/head/finalRegistration/", "meshOfFrame", transformation, 1);
		//auto input_mesh = std::make_shared<MeshReader>("../input_data/HaoLi/head/headInputScans/", "meshOfFrame", transformation, 0);
		//_data_name = "head";
		//_registration_options.use_vertex_random_probability = 0.3;
		//_registration_options.dg_options.edge_length = 0.2;
	
		// hand
		//auto reference_registration_mesh = std::make_unique<MeshReader>("../input_data/HaoLi/hand/hand1-registrationOutput/", "meshOfFrame", transformation, 1);
		//auto input_mesh = std::make_unique<MeshReader>("../input_data/HaoLi/hand/hand-inputScans/", "meshOfFrame", transformation, 0);		
		//_data_name = "hand";
		//_registration_options.dg_options.edge_length = 0.15;
		//_registration_options.use_vertex_random_probability = 0.5;

		if (register_on_reference_mesh || load_compare_mesh) {
			if (load_all_frames) {
				reference_registration_mesh->processAllFrames();
			}
			else {
				for (unsigned int i = 0; i < number_of_frames_to_load; i++) {
					reference_registration_mesh->processFrame();
				}
			}
		}
		if (!register_on_reference_mesh || load_compare_mesh) {
			if (load_all_frames) {
				input_mesh->processAllFrames();
			}
			else {
				for (unsigned int i = 0; i < number_of_frames_to_load; i++) {
					input_mesh->processFrame();
				}
			}
		}
		if (register_on_reference_mesh) {
			_input_mesh = std::move(reference_registration_mesh);
			_reference_registration_mesh = std::move(input_mesh);
		}
		else {
			_input_mesh = std::move(input_mesh);
			_reference_registration_mesh = std::move(reference_registration_mesh);
		}
		//_logger = std::make_shared<FileWriter>(_data_name + "_log.txt");
	}
	else {
		_registration_options.evaluate_residuals = true;
		_registration_options.dg_options.edge_length = 0.05;
		_registration_options.ignore_deformation_graph_border_vertices = false;
		_registration_options.dg_options.number_of_interpolation_neighbors = 4;
		_registration_options.use_vertex_random_probability = 1.;
		_registration_options.max_iterations = 50;
		_registration_options.smooth = 20.;
		_registration_options.fit = 10.;
		

		_input_mesh = std::make_shared<DeformationMeshFrames>();
		_reference_registration_mesh = std::make_shared<DeformationMeshFrames>();
		_renderer->_render_reference_mesh = false;
		_calculate_error = false;
		_renderer->_render_deformation_graph = true;
		_data_name = "test";
	}
	renderRegistration();


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
