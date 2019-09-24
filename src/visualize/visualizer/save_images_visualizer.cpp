#include "save_images_visualizer.h"

namespace Visualizer {


void SaveImagesVisualizer::visualize(Render mode)
{
	if (!finished()) {
		_mesh_visualizer->visualize(std::make_pair(_current_frame, ml::RGBColor::White), true);
	}
}

void SaveImagesVisualizer::registration()
{
	if (!finished()) {
		_current_frame++;		
		_image_name = "frame_" + std::to_string(_current_frame);
	}
}

bool SaveImagesVisualizer::finished()
{
	return _current_frame == _mesh_sequence->size();
}

void SaveImagesVisualizer::saveImage()
{
	if (_image_name != "") {
		_renderer->saveCurrentWindowAsImage(_save_images_folder, _image_name);
		_image_name = "";
	}
}

SaveImagesVisualizer::SaveImagesVisualizer(std::shared_ptr<IMeshReader> mesh_sequence,
										   std::shared_ptr<Renderer> renderer,
										   std::string image_folder_name)
	: _mesh_sequence(mesh_sequence)
	, _renderer(renderer)
	, _save_images_folder(image_folder_name)
{
	_mesh_visualizer = std::make_unique<MeshVisualizer>(_renderer, _mesh_sequence);
	_image_name = "frame_" + std::to_string(_current_frame);
}

}
