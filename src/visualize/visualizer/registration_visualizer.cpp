#include "registration_visualizer.h"


namespace Visualizer {

using namespace Registration;


void RegistrationVisualizer::registration()
{
	if (_registration) {
		bool finished = _registration->finished();
		if (!finished) {
			_registration->solveIteration();
			auto save_image = _registration->shouldBeSavedAsImage();
			if(save_image.first)
				_image_name = save_image.second;
		}
		else {
			std::cout << std::endl << "finished, select next two frames" << std::endl;
			_image_name = "frame_finished";
			_finished = true;
		}
	}
}

void RegistrationVisualizer::visualize(Render mode)
{
	if (!_finished && _registration) {
		auto deformed_points = _registration->getDeformedPoints();
		// defomed mesh
		_render_registration->renderDeformedSourceMesh(_registration->getDeformedPoints(), mode.mode, true);
		_render_registration->renderTargetMesh(_registration->getTarget(), mode.mode);

		auto non_rigid_registration = dynamic_cast<INonRigidRegistration*>(_registration.get()); // todo .... maybe external polymorthis
		if (non_rigid_registration) {
			// deformation graph
			auto deformation_graph = non_rigid_registration->getDeformationGraphMesh();
			setDeformationGraphColor(deformation_graph, Visualize::VertexColor::Default, _edge_color);
			_render_registration->renderDeformationGraph(deformation_graph, mode.mode);
		}
	}
}

bool RegistrationVisualizer::finished()
{
	return _finished;
}

void RegistrationVisualizer::saveImage()
{
	if (_image_name != "") {
		_renderer->saveCurrentWindowAsImage(_save_images_folder, _image_name);
		_image_name = "";
	}
}


RegistrationVisualizer::RegistrationVisualizer(std::unique_ptr<IRegistration> registration,
											   std::shared_ptr<Renderer> renderer,
											   std::string image_folder_name,
											   std::shared_ptr<FileWriter> logger,
											   Visualize::EdgeColor edge_coloring)
	: _registration(std::move(registration))
	, _renderer(renderer)
	, _save_images_folder(image_folder_name)
	, _logger(logger)
	, _edge_color(edge_coloring)
{
	_render_registration = std::make_unique<RendererRegistration>(_renderer);
}

}
