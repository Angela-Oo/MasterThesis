#include "parser.h"
#include "cxxopts/cxxopts.hpp"


Registration::Input inputById(std::string input_id)
{
	Registration::Input input;
	if (input_id == "head") {
		input.file_path = "../input_data/HaoLi/head/finalRegistration/";
		input.file_name = "meshOfFrame";
		input.start_index = 1;
		input.output_folder_name = input_id;
	}
	else if (input_id == "head_scan") {
		input.file_path = "../input_data/HaoLi/head/headInputScans/";
		input.file_name = "meshOfFrame";
		input.start_index = 0;
		input.output_folder_name = input_id;
	}
	else if (input_id == "hand")
	{
		input.file_path = "../input_data/HaoLi/hand/hand1-registrationOutput/";
		input.file_name = "meshOfFrame";
		input.start_index = 1;
		input.output_folder_name = input_id;
	}
	else if (input_id == "hand_scan")
	{
		input.file_path = "../input_data/HaoLi/hand/hand-inputScans/";
		input.file_name = "meshOfFrame";
		input.start_index = 0;
		input.output_folder_name = input_id;
	}
	else if (input_id == "puppet")
	{
		input.file_path = "../input_data/HaoLi/puppet/finalRegistration/";
		input.file_name = "mesh_1";
		input.start_index = 0;
		input.output_folder_name = input_id;
	}
	else if (input_id == "puppet_scan")
	{
		input.file_path = "../input_data/HaoLi/puppet/puppetInputScans/";
		input.file_name = "meshOfFrame";
		input.start_index = 0;
		input.output_folder_name = input_id;
	}
	else if (input_id == "paperbag")
	{
		input.file_path = "../input_data/HaoLi/paperbag/finalregistration/";
		input.file_name = "meshOfFrame";
		input.start_index = 1;
		input.output_folder_name = input_id;
	}
	else if (input_id == "paperbag_scan")
	{
		input.file_path = "../input_data/HaoLi/paperbag/inputscans/";
		input.file_name = "meshOfFrame";
		input.start_index = 1;
		input.output_folder_name = input_id;
	}
	return input;
}


namespace {

void parseSequence(cxxopts::ParseResult &result, Registration::RegistrationOptions &registration_options)
{
	if (result.count("s"))
	{
		registration_options.sequence_options.enable = true;
		if (result.count("init_rigid_deformation_with_non_rigid_globale_deformation"))
			registration_options.sequence_options.init_rigid_deformation_with_non_rigid_globale_deformation = result["init_rigid_with_non_rigid_deformation"].as<bool>();
		if (result.count("use_previous_frame"))
			registration_options.sequence_options.use_previous_frame_for_rigid_registration = result["use_previouse_frame"].as<bool>();
	}
}

void parseType(cxxopts::ParseResult &result, Registration::RegistrationOptions &registration_options)
{
	if (result.count("t"))
	{
		auto type = result["t"].as<std::string>();
		if (type == "ARAP")
			registration_options.type = Registration::RegistrationType::ARAP;
		else if (type == "ED")
			registration_options.type = Registration::RegistrationType::ED;
	}

	if (result.count("e"))
	{
		registration_options.deformation_graph.edge_length = result["e"].as<double>();
	}
}
	
void parseInput(cxxopts::ParseResult &result, Registration::RegistrationOptions &options)
{
	if (result.count("i"))
	{
		auto input = result["i"].as<std::string>();
		if (input == "head") {
			options.input_mesh_sequence.file_path = "../input_data/HaoLi/head/finalRegistration/";
			options.input_mesh_sequence.file_name = "meshOfFrame";
			options.input_mesh_sequence.start_index = 1;
			options.input_mesh_sequence.output_folder_name = input;
		}
		else if (input == "head_scan") {
			options.input_mesh_sequence.file_path = "../input_data/HaoLi/head/headInputScans/";
			options.input_mesh_sequence.file_name = "meshOfFrame";
			options.input_mesh_sequence.start_index = 0;
			options.input_mesh_sequence.output_folder_name = input;
		}
		else if (input == "hand")
		{
			options.input_mesh_sequence.file_path = "../input_data/HaoLi/hand/hand1-registrationOutput/";
			options.input_mesh_sequence.file_name = "meshOfFrame";
			options.input_mesh_sequence.start_index = 1;
			options.input_mesh_sequence.output_folder_name = input;
		}
		else if (input == "hand_scan")
		{
			options.input_mesh_sequence.file_path = "../input_data/HaoLi/hand/hand-inputScans/";
			options.input_mesh_sequence.file_name = "meshOfFrame";
			options.input_mesh_sequence.start_index = 0;
			options.input_mesh_sequence.output_folder_name = input;
		}
		else if (input == "puppet")
		{
			options.input_mesh_sequence.file_path = "../input_data/HaoLi/puppet/finalRegistration/";
			options.input_mesh_sequence.file_name = "mesh_1";
			options.input_mesh_sequence.start_index = 0;
			options.input_mesh_sequence.output_folder_name = input;
		}
		else if (input == "puppet_scan")
		{
			options.input_mesh_sequence.file_path = "../input_data/HaoLi/puppet/puppetInputScans/";
			options.input_mesh_sequence.file_name = "meshOfFrame";
			options.input_mesh_sequence.start_index = 0;
			options.input_mesh_sequence.output_folder_name = input;
		}
		else if (input == "paperbag")
		{
			options.input_mesh_sequence.file_path = "../input_data/HaoLi/paperbag/finalregistration/";
			options.input_mesh_sequence.file_name = "meshOfFrame";
			options.input_mesh_sequence.start_index = 1;
			options.input_mesh_sequence.output_folder_name = input;
		}
		else if (input == "paperbag_scan")
		{
			options.input_mesh_sequence.file_path = "../input_data/HaoLi/paperbag/inputscans/";
			options.input_mesh_sequence.file_name = "meshOfFrame";
			options.input_mesh_sequence.start_index = 1;
			options.input_mesh_sequence.output_folder_name = input;
		}
	}
	else {
		if (result.count("file_path") && result.count("file_name"))
		{
			options.input_mesh_sequence.file_path = result["file_path"].as<std::string>();
			options.input_mesh_sequence.file_name = result["file_name"].as<std::string>();
		}
		if (result.count("start_index"))
		{
			options.input_mesh_sequence.file_name = result["start_index"].as<unsigned int>();
		}
		if (result.count("output_folder_name"))
		{
			options.input_mesh_sequence.output_folder_name = result["output_folder_name"].as<std::string>();
		}
	}

	if (result.count("n"))
	{
		options.input_mesh_sequence.number_of_frames_to_load = result["n"].as<int>();
	}
	
	options.input_mesh_sequence.image_folder_name = result["image_folder_name"].as<std::string>();

	auto render_mode = result["render_mode"].as<std::string>();
	if (render_mode == "DEFORMATION")
		options.input_mesh_sequence.render_mode = Visualizer::DEFORMATION;
	else if(render_mode == "TARGET")
		options.input_mesh_sequence.render_mode = Visualizer::TARGET;
	else if(render_mode == "ALL")
		options.input_mesh_sequence.render_mode = Visualizer::ALL;
	else if (render_mode == "DEFORMATION_GRAPH")
		options.input_mesh_sequence.render_mode = Visualizer::ONLY_DEFORMATION_GRAPH;

	options.input_mesh_sequence.term = result["term"].as<bool>();
}

void parseRefinement(Registration::RegistrationOptions& registration_options, cxxopts::ParseResult result)
{
	if (result.count("r"))
	{
		registration_options.refinement.enable = result["r"].as<bool>();
		if(result["refine_at_edge"].as<bool>())
			registration_options.refinement.refine = Registration::RefinementOptions::Refinement::EDGE;
		else
			registration_options.refinement.refine = Registration::RefinementOptions::Refinement::VERTEX;
		registration_options.refinement.smooth_cost_threshold = result["smooth_cost_threshold"].as<double>();
	}
}


void parseAdaptiveRigidity(Registration::RegistrationOptions& registration_options, cxxopts::ParseResult result)
{
	if (result["adaptive_rigidity"].as<bool>()) {
		registration_options.adaptive_rigidity.enable = true;
		registration_options.adaptive_rigidity.rigidity_cost_coefficient = result["rigidity_cost_coefficient"].as<double>();
	}
}

void parseReduceRigidity(Registration::RegistrationOptions& registration_options, cxxopts::ParseResult result)
{
	if (result["reduce_rigidity"].as<bool>()) {
		registration_options.reduce_rigidity.enable = true;
		registration_options.reduce_rigidity.rigidity_cost_threshold = result["rigidity_cost_threshold"].as<double>();
		registration_options.reduce_rigidity.minimal_rigidity = result["minimal_rigidity"].as<double>();
	}
}

void parseOptions(Registration::RegistrationOptions& registration_options, cxxopts::ParseResult result)
{
	registration_options.max_iterations = result["max_iterations"].as<unsigned int>();
	registration_options.use_vertex_random_probability = result["p"].as<double>();
	registration_options.smooth = result["smooth"].as<double>();
	registration_options.fit = result["fit"].as<double>();
	registration_options.ignore_border_vertices = result["ignore_border_vertices"].as<bool>();

	registration_options.error_evaluation = result["error_evaluation"].as<bool>();
}
}



Registration::RegistrationOptions parse(int argc, char* argv[])
{
	using namespace Registration;
	try
	{
		Registration::RegistrationOptions registration_options;

		cxxopts::Options options(argv[0], " - example command line options");
		options
			.positional_help("[optional args]")
			.show_positional_help();

		options
			.allow_unrecognised_options()
			.add_options()
			("t,type", "Registration Type: ARAP, ED", cxxopts::value<std::string>(std::string{ "ARAP" }))
			("e,edge_length", "Deformation Graph Edge length", cxxopts::value<double>()->default_value("0.3"));

		options
			.add_options()
			("i,input", "Select Input Mesh Sequence by name: head, hand, paperbag, puppet", cxxopts::value<std::string>()->default_value("head"))
			("n,number_of_frames_to_load", "Number of Meshes to Load (-1 for all)", cxxopts::value<int>()->default_value("-1"))
			("file_path", "Input Meshes File Path (has only effect if 'input' is not given)", cxxopts::value<std::string>()->default_value("../input_data/HaoLi/head/finalRegistration/"))
			("file_name", "Input Meshes File Name (has only effect if 'input' is not given)", cxxopts::value<std::string>()->default_value("meshOfFrame"))
			("start_index", "Input Meshes Start File Index (has only effect if 'input' is not given)", cxxopts::value<unsigned int>()->default_value("1"))
			("output_folder_name", "Output Folder Name (has only effect if 'input' is not given)", cxxopts::value<std::string>()->default_value("head"))
			("image_folder_name", "Image Folder Name", cxxopts::value<std::string>()->default_value("images"))
			("render_mode", 
			 "Image Render Mode: DEFORMATION (deformation, deformation_graph), ALL (deformation, target, deformation_graph), TARGET (target, deformation_graph), DEFORMATION_GRAPH (deformation_graph)",
			 cxxopts::value<std::string>()->default_value("DEFORMATION"))
			("term", "Terminates the application after the registration finished", cxxopts::value<bool>()->default_value("true"));

		options.add_options()
			("error_evaluation", "Enable disable error evaluation", cxxopts::value<bool>()->default_value("true"));

		options
			.add_options()
			("s,sequence", "Sequence Registration")
			("init_rigid_with_non_rigid_deformation", "Init Rigid Deformation With Non Rigid Global Deformation (has only effect if 'sequence')", cxxopts::value<bool>()->default_value("true"))
			("use_previous_frame", "Use Previous Frame for Rigid Registration (has only effect if 'sequence')", cxxopts::value<bool>()->default_value("true"));

		options.add_options()
			("rigid_and_non_rigid", "Rigid before Non-Rigid Refinement (default true)", cxxopts::value<bool>()->default_value("true"));

		options.add_options()
			("adaptive_rigidity", "Adaptive Rigidity")
			("rigidity_cost_coefficient", "Adaptive Rigidity cost coefficient", cxxopts::value<double>()->default_value("0.005"));			

		options.add_options()
			("reduce_rigidity", "Reduce Rigidity")
			("rigidity_cost_threshold", "If smooth cost is bigger than threshold -> reduce rigidity", cxxopts::value<double>()->default_value("0.01"))
			("minimal_rigidity", "Minimal rigidity value", cxxopts::value<double>()->default_value("0.1"));

		options.add_options()
			("r,refine_deformation_graph", "Deformation Graph Refinement")
			("refine_at_edge", "Refine Deformation Graph At Edge (default is Vertex)")
			("smooth_cost_threshold", "Refine Deformation Graph if smooth cost of at edge or vertex is bigger than threshold", cxxopts::value<double>()->default_value("0.05"));

		options.add_options()
			("max_iterations", "Max Iterations", cxxopts::value<unsigned int>()->default_value("25"))
			("p,probability_to_use_vertex", "Random Probability to use a input vertex", cxxopts::value<double>()->default_value("0.2"))
			("smooth", "Smooth coefficient used for registration", cxxopts::value<double>()->default_value("1."))
			("fit", "Fit coefficient used for registration", cxxopts::value<double>()->default_value("1."))
			("ignore_border_vertices", "Ignore Border Vertices of target mesh", cxxopts::value<bool>()->default_value("true"));

		auto result = options.parse(argc, argv);

		if (result.count("help"))
		{
			std::cout << options.help({ "", "Group" }) << std::endl;
			exit(0);
		}

		parseType(result, registration_options);
		parseSequence(result, registration_options);
		parseInput(result, registration_options);

		if (result.count("rigid_and_non_rigid"))
		{
			registration_options.rigid_and_non_rigid_registration = result["rigid_and_non_rigid"].as<bool>();
		}
		
		parseRefinement(registration_options, result);
		parseAdaptiveRigidity(registration_options, result);
		parseReduceRigidity(registration_options, result);

		parseOptions(registration_options, result);

		std::cout << "Arguments remain = " << argc << std::endl;

		return registration_options;

	}
	catch (const cxxopts::OptionException& e)
	{
		std::cout << "error parsing options: " << e.what() << std::endl;
		exit(1);
	}
}

