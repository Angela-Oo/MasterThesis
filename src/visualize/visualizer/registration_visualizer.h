#pragma once

#include "i_registration_visualizer.h"
#include "visualize/render/renderer.h"
#include "algo/registration/interface/i_registration.h"
#include "util/file_writer.h"
#include "algo/registration_evaluation/evaluate_registration.h"
#include "visualize/render/render_sequence_registration.h"
#include <memory>

namespace Visualizer {

class RegistrationVisualizer : public IRegistrationVisualizer
{
private:
	std::unique_ptr<Registration::IRegistration> _registration;
	std::shared_ptr<Renderer> _renderer;
	std::shared_ptr<FileWriter> _logger;
	std::unique_ptr<ErrorEvaluation> _error_evaluation;
	std::unique_ptr<RendererRegistration> _render_registration;
	std::string _save_images_folder;
	std::string _image_name;
	bool _finished{ false };
private:
	void renderError();
	void renderRegistration(RegistrationRenderMode mode);
public:
	void registration() override;
	void visualize(RegistrationRenderMode mode, bool visible) override;
	bool finished() override;
	void saveImage() override;
public:
	RegistrationVisualizer(std::unique_ptr<Registration::IRegistration> registration,
						   std::shared_ptr<Renderer> renderer,
						   std::string image_folder_name,
						   std::shared_ptr<FileWriter> logger);
};



}