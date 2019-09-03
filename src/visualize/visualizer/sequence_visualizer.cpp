#include "sequence_visualizer.h"

#include "visualize/render/render_deformation_graph.h"

namespace Visualizer {

using namespace Registration;

void SequenceRegistrationVisualizer::renderError()
{
}

void SequenceRegistrationVisualizer::renderRegistration(Render mode)
{
	if (_registration) {
		auto current = _registration->getCurrent();

		if (_finished) {
			//_renderer->remove("target");
			//_renderer->remove("deformed");
			//_renderer->remove("inverse_mesh");
			//_renderer->remove("mesh_source");
		
			_render_registration->renderDeformedSourceMesh(_registration->getDeformedMesh(_current_frame), mode.mode);
			_render_registration->renderTargetMesh(_registration->getMesh(_current_frame), mode.mode);
			_render_registration->renderDeformationGraph(_registration->getDeformationGraphMesh(_current_frame), mode.mode);

/*			if (_mode == Render::DEFORMATION) {
				auto inverse_deformed_points = _registration->getInverseDeformedMesh(_current_frame);
				_renderer->insert("inverse_mesh", inverse_deformed_points,  ml::RGBColor::Cyan, false);
				auto source = _registration->getMesh(0);
				_renderer->insertMesh("mesh_source", source, ml::RGBColor::Yellow.toVec4f(), false);
			}	*/		
		}
		else {
			_finished = _registration->finished();

			_render_registration->renderDeformedSourceMesh(_registration->getDeformedMesh(_current_frame), mode.mode);
			_render_registration->renderTargetMesh(_registration->getMesh(_current_frame), mode.mode);
			_render_registration->renderDeformationGraph(_registration->getDeformationGraphMesh(_current_frame), mode.mode);
		}
	}
}

void SequenceRegistrationVisualizer::visualize(Render mode)
{
	if (!_finished) {
		renderRegistration(mode);
		renderError();
	}
}

void SequenceRegistrationVisualizer::registration()
{
	if (_registration)
	{
		bool finished = _registration->finished();
		if (!finished) {
			bool iteration_finished = _registration->solve();
			auto save_image = _registration->saveCurrentFrameAsImage();
			if (save_image.first)
				_image_name = "frame_" + save_image.second;
		}
		else {
			std::cout << std::endl << "finished registration" << std::endl;
			_image_name = "frame_finished";
			auto render_mode = RegistrationRenderMode::NONE;
			_finished = true;
		}
	}
}

bool SequenceRegistrationVisualizer::finished()
{
	return _finished;
}

void SequenceRegistrationVisualizer::saveImage()
{
	if (_image_name != "") {
		_renderer->saveCurrentWindowAsImage(_save_images_folder, _image_name);
		_image_name = "";
	}
}

SequenceRegistrationVisualizer::SequenceRegistrationVisualizer(std::unique_ptr<Registration::ISequenceRegistration> registration,
															   std::shared_ptr<Renderer> renderer,
															   std::string image_folder_name,
															   std::shared_ptr<FileWriter> logger)
	: _registration(std::move(registration))
	, _renderer(renderer)
	, _save_images_folder(image_folder_name)
	, _logger(logger)
{
	_render_registration = std::make_unique<RendererRegistration>(_renderer);
}

}
