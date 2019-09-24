#pragma once

#include "i_registration_visualizer.h"
#include "visualize/render/renderer.h"
#include <memory>
#include "mesh/i_mesh_sequence.h"
#include "visualize/showMesh.h"

namespace Visualizer {

class SaveImagesVisualizer : public IRegistrationVisualizer
{
private:
	std::shared_ptr<IMeshReader> _mesh_sequence;
	std::shared_ptr<Renderer> _renderer;
	std::shared_ptr<MeshVisualizer> _mesh_visualizer;
	std::string _save_images_folder;
	std::string _image_name {""};
	unsigned int _current_frame{0};
public:
	void registration() override;
	void visualize(Render mode) override;
	bool finished() override;
	void saveImage() override;
public:
	SaveImagesVisualizer(std::shared_ptr<IMeshReader> mesh_sequence,
						 std::shared_ptr<Renderer> renderer,
						 std::string image_folder_name);
};



}
