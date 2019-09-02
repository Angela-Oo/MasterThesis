#include "create_registration_visualizer.h"

#include "sequence_visualizer.h"
#include "registration_visualizer.h"
#include "algo/registration/interface/registration.h"

#include "image_folder_name.h"
#include <memory>

namespace Visualizer
{

std::shared_ptr<IRegistrationVisualizer> createRegistrationVisualizer(std::shared_ptr<Renderer> renderer,
																	  std::shared_ptr<IMeshReader> mesh_reader,
																	  Registration::RegistrationOptions & options)
{
	auto _save_images_folder = imageFolderName(options);
	auto logger = std::make_shared<FileWriter>(_save_images_folder + "/" + options.input_mesh_sequence.output_folder_name + "_log.txt");

	if (options.sequence_options.enable) {
		auto register_sequence_of_frames = Registration::createSequenceRegistration(options, logger, mesh_reader);
		return std::make_shared<SequenceRegistrationVisualizer>(std::move(register_sequence_of_frames), renderer, logger);
	}
	else {
		auto & source = mesh_reader->getMesh(0);
		auto & target = mesh_reader->getMesh(mesh_reader->size() - 1);

		auto registration = Registration::createRegistration(options, logger, source, target);
		return std::make_shared<RegistrationVisualizer>(std::move(registration), renderer, logger);
		//_image_name = "frame_" + std::to_string(registration->currentIteration());
	}
}

}