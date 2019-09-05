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
		if (result.count("use_previouse_frame"))
			registration_options.sequence_options.use_previouse_frame_for_rigid_registration = result["use_previouse_frame"].as<bool>();
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
			("output_folder_name", "Output Folder Name (has only effect if 'input' is not given)", cxxopts::value<std::string>()->default_value("head"));

		options
			.add_options()
			("s,sequence", "Sequence Registration")
			("init_rigid_with_non_rigid_deformation", "Init Rigid Deformation With Non Rigid Global Deformation (has only effect if 'sequence')", cxxopts::value<bool>()->default_value("true"))
			("use_previouse_frame", "Use Previouse Frame for Rigid Registration (has only effect if 'sequence')", cxxopts::value<bool>()->default_value("true"));

		options.add_options()
			("rigid_and_non_rigid", "Rigid before Non-Rigid Refinement (default true)", cxxopts::value<bool>()->default_value("true"))
			("a,adaptive_rigidity", "Adaptive Rigidity: REDUCE_RIGIDITY, RIGIDITY_COST", cxxopts::value<std::string>());

		options.add_options()
			("r,refine_deformation_graph", "Deformation Graph Refinement")
			("refine_at_edge", "Refine Deformation Graph At Edge (default is Vertex)");

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
		if (result.count("r"))
		{
			registration_options.refinement.enable = result["r"].as<bool>();
			if(result["refine_at_edge"].as<bool>())
				registration_options.refinement.refine = RefinementOptions::Refinement::EDGE;
			else
				registration_options.refinement.refine = RefinementOptions::Refinement::VERTEX;
		}
		if (result.count("a"))
		{
			registration_options.adaptive_rigidity.enable = result["a"].as<bool>();
		}

		registration_options.max_iterations = result["max_iterations"].as<unsigned int>();
		registration_options.use_vertex_random_probability = result["p"].as<double>();
		registration_options.smooth = result["smooth"].as<double>();
		registration_options.fit = result["fit"].as<double>();
		registration_options.ignore_border_vertices = result["ignore_border_vertices"].as<bool>();

		std::cout << "Arguments remain = " << argc << std::endl;

		return registration_options;

	}
	catch (const cxxopts::OptionException& e)
	{
		std::cout << "error parsing options: " << e.what() << std::endl;
		exit(1);
	}
}


