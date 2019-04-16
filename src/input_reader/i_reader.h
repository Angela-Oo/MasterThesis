#pragma once
#include "../mLibInclude.h"

class IReader
{
public:
	virtual std::vector<ml::vec3f> getPoints(unsigned int frame, unsigned int step_size = 1) = 0;
	virtual void processFrame() = 0;
	virtual unsigned int frame() = 0;
public:
	virtual void load(std::string filename) = 0;
	virtual void save(std::string filename) = 0;
public:
	virtual ~IReader() = default;
};
