#include "parser.h"
#include "cxxopts/cxxopts.hpp"

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
			("t,type", "Registration Type: ARAP, ED", cxxopts::value<std::string>(std::string{ "ARAP" }));

		options.add_options()
			("s,sequence", "Sequence Registration")
			("init_rigid_with_non_rigid_deformation", "Init Rigid Deformation With Non Rigid Global Deformation (has only effect if 'sequence')", cxxopts::value<bool>()->default_value("true"))
			("use_previouse_frame", "Use Previouse Frame for Rigid Registration (has only effect if 'sequence')", cxxopts::value<bool>()->default_value("true"));

		options.add_options()
			("rigid_and_non_rigid", "Rigid before Non-Rigid Refinement (default true)", cxxopts::value<bool>()->default_value("true"))
			("r,refine_deformation_graph", "Deformation Graph Refinement")
			("a,adaptive_rigidity", "Adaptive Rigidity: REDUCE_RIGIDITY, RIGIDITY_COST", cxxopts::value<std::string>());

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

		if (result.count("t"))
		{
			auto type = result["t"].as<std::string>();
			if (type == "ARAP")
				registration_options.type = Registration::RegistrationType::ARAP;
			else if(type == "ED")
				registration_options.type = Registration::RegistrationType::ED;
		}

		if (result.count("s"))
		{
			registration_options.sequence_options.enable = true;
			if(result.count("init_rigid_deformation_with_non_rigid_globale_deformation"))
				registration_options.sequence_options.init_rigid_deformation_with_non_rigid_globale_deformation = result["init_rigid_with_non_rigid_deformation"].as<bool>();
			if (result.count("use_previouse_frame"))
				registration_options.sequence_options.use_previouse_frame_for_rigid_registration = result["use_previouse_frame"].as<bool>();
		}

		if (result.count("rigid_and_non_rigid"))
		{
			registration_options.rigid_and_non_rigid_registration = result["rigid_and_non_rigid"].as<bool>();;
		}

		std::cout << "Arguments remain = " << argc << std::endl;

		return registration_options;

	}
	catch (const cxxopts::OptionException& e)
	{
		std::cout << "error parsing options: " << e.what() << std::endl;
		exit(1);
	}
}
