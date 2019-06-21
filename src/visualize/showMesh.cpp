#include "stdafx.h"
#include "showMesh.h"
#include <algorithm>
#include <cmath>

#include "algo/registration/rigid_registration/rigid_registration.h"
#include "algo/registration/ceres_option.h"

#include "algo/registration/embedded_deformation/ed.h"
#include "algo/registration/arap/arap.h"

#include "algo/surface_mesh/mesh_convertion.h"

#include <chrono>  // chrono::system_clock
#include <ctime>   // localtime
#include <iomanip> // put_time

std::string ShowMesh::getImageFolderName(RegistrationType type)
{
	auto in_time_t = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
	std::stringstream ss;
	ss << std::put_time(std::localtime(&in_time_t), "%d-%m-%Y_%H-%M");
	std::string folder_name = "registration_";
	if (type == RegistrationType::ARAP) {
		folder_name = "ASAP_";
	}
	else if (type == RegistrationType::ED) {
		folder_name = "ED_";
	}
	else if (type == RegistrationType::Rigid) {
		folder_name = "Rigid_";
	}
	else if (type == RegistrationType::ARAP_AllFrames) {
		folder_name = "ARAP_AllFrames";
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
		bool evaluate_residuals = false;

		_save_images_folder = getImageFolderName(_registration_type);
		_logger = std::make_shared<FileWriter>(_save_images_folder + "/" + _data_name + "_log.txt");

		_registration = createRegistration(source, target, _registration_type, option, evaluate_residuals, _logger, _deformation_graph_edge_length, _input_mesh->getFixedPositions(frame_b));
		renderRegistration();
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
		std::vector<SurfaceMesh> meshes;
		for (unsigned int i = 0; i < _input_mesh->frame(); ++i) {
			meshes.push_back(_input_mesh->getMesh(i));
		}
		RegistrationType type = _registration_type == RegistrationType::ARAP_AllFrames ? RegistrationType::ARAP : RegistrationType::ED;

		_save_images_folder = getImageFolderName(_registration_type);
		_logger = std::make_shared<FileWriter>(_save_images_folder + "/" + _data_name + "_log.txt");
		_register_sequence_of_frames = std::make_unique<SequenceRegistration>(meshes, type, _logger, _deformation_graph_edge_length);
	}
	else {
		bool finished = _register_sequence_of_frames->finished();
		if (!finished) {
			bool iteration_finished = _register_sequence_of_frames->solve();
			if(iteration_finished)
				_image_name = "frame_" + std::to_string(_register_sequence_of_frames->getCurrent());
		}
		else {		
			std::cout << std::endl << "finished registration" << std::endl;
			_solve_registration = false;
			_image_name = "frame_finished";
		}
	}
}

void ShowMesh::renderError()
{	
	if (_registration && _calculate_error) {
		if (!_error_evaluation) {
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
	_renderer->renderCurrentFrame(_input_mesh, _current_frame, render_current_frame);

	// selected frames
	auto selected_frames = render_current_frame ? _selected_frame_for_registration : std::vector<unsigned int>();
	_renderer->renderSelectedFrames(_input_mesh, selected_frames);	

	// reference
	_renderer->renderReference(_reference_registration_mesh, _current_frame);
}

void ShowMesh::renderRegistration()
{
	renderCurrentMesh();
	_renderer->renderRegistration(_registration);
	_renderer->renderRegistrationSequence(_register_sequence_of_frames);
	renderError();
}

void ShowMesh::render(ml::Cameraf& camera)
{
	if (_solve_registration && _register_sequence_of_frames) {
		solveAllNonRigidRegistration();	
		_current_frame = _register_sequence_of_frames->getCurrent();
	}
	else if (_solve_registration && _registration && _selected_frame_for_registration.size() == 2) {
		nonRigidRegistration();
	}
	renderRegistration();
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
		_current_frame++;
		if (_current_frame >= _input_mesh->frame())
			_current_frame = 0;
		renderRegistration();
	}
	else if (key == KEY_1)
	{
		if (_current_frame == 0)
			_current_frame = _input_mesh->frame() - 1;
		else
			_current_frame--;
		renderRegistration();
	}
	else if (key == KEY_I)
	{
		if (_selected_frame_for_registration.size() < 2) {
			if (_selected_frame_for_registration.empty())
				_registration.reset();
			if (_selected_frame_for_registration.empty() || (_selected_frame_for_registration.size() == 1 && _selected_frame_for_registration[0] != _current_frame)) {
				std::cout << "select frame " << _current_frame << " for registration" << std::endl;
				_selected_frame_for_registration.push_back(_current_frame);
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
	else if (key == KEY_U)
	{
		std::cout << "register all frames as rigid as possible " << std::endl;
		if (!_register_sequence_of_frames) {
			_current_frame = 0;
			_registration_type = RegistrationType::ARAP_AllFrames;
			solveAllNonRigidRegistration();			
			_solve_registration = true;
		}
	}
	else if (key == KEY_P)
	{
		std::cout << "register all frames embdedded deformation " << std::endl;
		if (!_register_sequence_of_frames) {
			_current_frame = 0;
			_registration_type = RegistrationType::ED_AllFrames;
			solveAllNonRigidRegistration();
			_solve_registration = true;
		}
	}
	else if (key == KEY_T)
	{		
		if (!_registration) {
			std::cout << "test registration " << std::endl;
			_current_frame = 1;
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
		std::string visible = (mode != Render::NONE) ? "show" : "hide";
		std::cout << visible << " mesh" << std::endl;
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
	//_number_of_nodes = 1500;
	_deformation_graph_edge_length = 0.05;
	_current_frame = 0;
	_solve_registration = false;
	_registration_type = RegistrationType::ARAP;
	_calculate_error = false;
	_renderer = std::make_unique<RenderRegistration>(&app.graphics);
	//_saved_image = 0;

	ml::mat4f scale = ml::mat4f::scale(0.01);
	ml::mat4f rotation = ml::mat4f::rotationX(-90.);
	ml::mat4f transform = ml::mat4f::translation({ -0.5f, 3.5f, 1.5f });
	ml::mat4f transform2 = ml::mat4f::translation({ 0.f, -10.f, 0.0f });
	ml::mat4f transformation = transform2 * transform * rotation * scale;

	bool test = false;
	if (!test) {
		// puppet
		auto reference_registration_mesh = std::make_unique<MeshReader>("../input_data/HaoLi/puppet/finalRegistration/", "mesh_1",  transformation, 1);
		auto input_mesh = std::make_unique<MeshReader>("../input_data/HaoLi/puppet/puppetInputScans/", "meshOfFrame", transformation, 0);
		_deformation_graph_edge_length = 0.07;
		_data_name = "puppet";

		// paperbag
		//auto reference_registration_mesh = std::make_unique<MeshReader>("../input_data/HaoLi/paperbag/finalregistration/", "meshOfFrame", transformation, 1);
		//auto input_mesh = std::make_unique<MeshReader>("../input_data/HaoLi/paperbag/inputscans/", "meshOfFrame", transformation, 1);
		//_deformation_graph_edge_length = 0.1;
		//_data_name = "paperbag";

		// head
		//auto reference_registration_mesh = std::make_shared<MeshReader>("../input_data/HaoLi/head/finalRegistration/", "meshOfFrame", transformation, 1);
		//auto input_mesh = std::make_shared<MeshReader>("../input_data/HaoLi/head/headInputScans/", "meshOfFrame", transformation, 0);
		//_deformation_graph_edge_length = 0.04;
		//_data_name = "head";
	
		// hand
/*		auto reference_registration_mesh = std::make_unique<MeshReader>("../input_data/HaoLi/hand/hand1-registrationOutput/", "meshOfFrame", transformation, 1);
		auto input_mesh = std::make_unique<MeshReader>("../input_data/HaoLi/hand/hand-inputScans/", "meshOfFrame", transformation, 0);		
		_data_name = "hand";	*/	

		input_mesh->processAllFrames();
		reference_registration_mesh->processAllFrames();
		//for (int i = 0; i < 20; i++) {
		//	input_mesh->processFrame();
		//	reference_registration_mesh->processFrame();
		//}
		_input_mesh = std::move(input_mesh);
		_reference_registration_mesh = std::move(reference_registration_mesh);
		//_logger = std::make_shared<FileWriter>(_data_name + "_log.txt");
	}
	else {
		//_input_mesh = std::make_unique<DeformationMesh>();
		//_reference_registration_mesh = std::make_unique<DeformationMesh>();

		_input_mesh = std::make_shared<DeformationMeshFrames>();
		_reference_registration_mesh = std::make_shared<DeformationMeshFrames>();

		//_logger = std::make_shared<FileWriter>("test.txt");
		_renderer->_render_reference_mesh = false;
		_calculate_error = false;
		_renderer->_render_deformation_graph = true;
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
	std::cout << "    non rigid registration for all frames (arap):  KEY_U" << std::endl;
	std::cout << "    non rigid registration for all frames (ed):    KEY_P" << std::endl;

}
