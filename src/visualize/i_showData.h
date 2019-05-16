#pragma once
#include "../mLibInclude.h"

class IShowData
{
public:
	virtual void init(ml::ApplicationData &app) = 0;
	virtual void render(ml::Cameraf& camera) = 0;
	virtual void key(UINT key) = 0;
	virtual ~IShowData() = default;
};
