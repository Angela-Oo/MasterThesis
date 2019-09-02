#pragma once

#include "visualize/render/renderer.h"
#include "mesh/i_mesh_sequence.h"
#include <memory>

namespace Visualizer {

class MeshVisualizer
{
private:
	std::shared_ptr<Renderer> _renderer;
	std::shared_ptr<IMeshReader> _mesh_reader;
	unsigned int _last_rendered_current_frame;
	unsigned int _current_frame;
public:
	void visualize(bool visible);
	void setCurrentFrame(unsigned int current_frame);
public:
	MeshVisualizer(std::shared_ptr<Renderer> renderer,
				   std::shared_ptr<IMeshReader> mesh_reader);
};

}