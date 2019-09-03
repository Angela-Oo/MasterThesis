#pragma once

#include "i_registration_visualizer.h"
#include "visualize/render/renderer.h"
#include <memory>
#include "algo/registration/interface/i_sequence_registration.h"

#include "visualize/render/render_sequence_registration.h"
#include "util/file_writer.h"

namespace Visualizer {

class SequenceRegistrationVisualizer : public IRegistrationVisualizer
{
private:
	std::unique_ptr<Registration::ISequenceRegistration> _registration;
	std::shared_ptr<Renderer> _renderer;
	std::shared_ptr<FileWriter> _logger;
	//std::unique_ptr<ErrorEvaluation> _error_evaluation;


	std::unique_ptr<RendererRegistration> _render_registration;
	std::string _save_images_folder;
	std::string _image_name;
	bool _finished{ false };
private:
	unsigned int _current_frame;
private:
	void renderError();
	void renderRegistration(RegistrationRenderMode mode);
public:
	void registration() override;
	void visualize(RegistrationRenderMode mode, bool visible) override;
	bool finished() override;
	void saveImage() override;
public:
	SequenceRegistrationVisualizer(std::unique_ptr<Registration::ISequenceRegistration> registration,
								   std::shared_ptr<Renderer> renderer,
								   std::string image_folder_name,
								   std::shared_ptr<FileWriter> logger);
};


}