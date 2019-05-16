#pragma once
#include "../mLibInclude.h"

struct ConstantBuffer
{
	ml::mat4f worldViewProj;
	ml::vec4f modelColor;
	ml::vec4f lightDir;
	ml::vec4f eye;
	//ml::vec1f ambientLight;
	//ml::vec1f diffuseLight;
	ConstantBuffer()
		: worldViewProj(ml::mat4f::identity())
		, modelColor(ml::vec4f(1.0f, 1.0f, 1.0f, 1.0f))
		, lightDir(ml::vec4f(3., -1., 5., 0.).getNormalized())
		, eye(ml::vec4f::origin)
		//, ambientLight(ml::vec1f(0.3f))
		//, diffuseLight(ml::vec1f(0.3f))
	{}
};