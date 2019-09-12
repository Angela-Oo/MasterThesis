#include "image_folder_name.h"

#include <chrono>  // chrono::system_clock
#include <ctime>   // localtime
#include <iomanip> // put_time
#include <sstream>

std::string imageFolderName(Registration::RegistrationOptions options)
{
	using namespace Registration;
	
	std::string folder_name = "registration_";

	if (options.type == RegistrationType::ARAP) {
		folder_name = "ARAP_";
	}
	else if (options.type == RegistrationType::ED) {
		folder_name = "ED_";
	}
	else if (options.type == RegistrationType::Rigid) {
		folder_name = "Rigid_";
	}

	if (options.sequence_options.enable)
	{
		folder_name += "AllFrames_";
	}

	if (!options.icp.enable)
	{
		folder_name += "WithoutICP_";
	}

	if (options.adaptive_rigidity.enable) 
	{
		folder_name += "AdaptiveRigidity_";
		if (options.adaptive_rigidity.refinement == Refinement::VERTEX)
			folder_name += "Vertex_";
		else
			folder_name += "Edge_";
	}
	
	if (options.reduce_rigidity.enable) 
	{
		folder_name += "ReduceRigidity_";
	}

	if (options.refinement.enable)
	{
		folder_name += "Refinement_";
		if (options.refinement.refine == Refinement::VERTEX)
			folder_name += "Vertex_";
		else
			folder_name += "Edge_";
	}
	

	auto in_time_t = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
	std::stringstream ss;
	ss << std::put_time(std::localtime(&in_time_t), "%Y-%m-%d_%H-%M");

	return options.input_mesh_sequence.image_folder_name + "/" + options.input_mesh_sequence.output_folder_name + "/" + folder_name + ss.str();
}