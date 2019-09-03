#pragma once

#include "visualize/render/renderer.h"
#include "mesh/i_mesh_sequence.h"
#include <memory>
#include <set>

namespace Visualizer {

class MeshVisualizer
{
private:
	std::shared_ptr<Renderer> _renderer;
	std::shared_ptr<IMeshReader> _mesh_reader;
	std::set<unsigned int> _visible_frames;
public:
	void visualize(std::pair<unsigned int, ml::RGBColor> frame, bool visible);
	void visualize(std::vector<std::pair<unsigned int, ml::RGBColor>> frames, bool visible);
	void clear();
public:
	MeshVisualizer(std::shared_ptr<Renderer> renderer,
				   std::shared_ptr<IMeshReader> mesh_reader);
};

}