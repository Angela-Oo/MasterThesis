#include "mesh_visualizer.h"

namespace Visualizer {



void MeshVisualizer::visualize(std::pair<unsigned int, ml::RGBColor> frame, bool visible)
{
	visualize(std::vector<std::pair<unsigned int, ml::RGBColor>>{ frame }, visible);
}

void MeshVisualizer::visualize(std::vector<std::pair<unsigned int, ml::RGBColor>> frames, bool visible)
{	
	auto id = [](const unsigned int & frame) { return "mesh" + std::to_string(frame); };
	if (visible)
	{
		for (auto it = _visible_frames.begin(); it != _visible_frames.end();) {
			auto & visible_frame = *it;
			auto found = std::find_if(frames.begin(), frames.end(), [&](auto f) { return f.first == visible_frame.first && f.second == visible_frame.second; });
			if (found == frames.end()) {
				_renderer->remove(id(visible_frame.first));
				_visible_frames.erase(it++);
			}
			else {
				++it;
			}
		}


		for (auto frame : frames) {			
			////std::find(_visible_frames.begin(), _visible_frames.end(), frame.first) == _visible_frames.end();
			bool override_mesh = _visible_frames.find(frame.first) == _visible_frames.end();
			_renderer->insert(id(frame.first), _mesh_reader->getMesh(frame.first), RenderMode::MESH, frame.second, override_mesh);
			if(override_mesh)
				_visible_frames.insert(frame);
		}
	}
	else {
		clear();
	}
}

void MeshVisualizer::clear()
{
	auto id = [](auto frame) { return "mesh" + std::to_string(frame); };
	for (auto visible_frame : _visible_frames)
		_renderer->remove(id(visible_frame.first));
	_visible_frames.clear();
}

MeshVisualizer::MeshVisualizer(std::shared_ptr<Renderer> renderer,
							   std::shared_ptr<IMeshReader> mesh_reader)
	: _renderer(renderer)
	, _mesh_reader(mesh_reader)
{}

}
