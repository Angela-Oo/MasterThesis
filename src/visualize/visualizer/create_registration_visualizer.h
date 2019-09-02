#pragma once

#include "i_registration_visualizer.h"
#include "mesh/i_mesh_sequence.h"
#include "visualize/render/renderer.h"
#include "algo/registration/interface/registration_options.h"
#include <memory>

namespace Visualizer
{

std::shared_ptr<IRegistrationVisualizer> createRegistrationVisualizer(std::shared_ptr<Renderer> renderer,
																	  std::shared_ptr<IMeshReader> mesh_reader,
																	  Registration::RegistrationOptions & options);

}