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


class IRegistrationVisualizer
{
public:
	virtual void registration() = 0;
	virtual void visualize(RegistrationRenderMode mode, bool visible) = 0;
	virtual bool finished() = 0;
	virtual void saveImage() = 0;
};

}