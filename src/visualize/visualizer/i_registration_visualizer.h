#pragma once

namespace Visualizer {


enum RegistrationRenderMode
{
	NONE,
	ONLY_DEFORMATION_GRAPH,
	DEFORMATION,
	TARGET,
	ALL
};

struct Render
{
	RegistrationRenderMode mode;
	bool show_deformation_graph { true };
	bool show_error{ false };
	bool show_reference{ false };
};

class IRegistrationVisualizer
{
public:
	virtual void registration() = 0;
	virtual void visualize(Render mode) = 0;
	virtual bool finished() = 0;
	virtual void saveImage() = 0;
};

}