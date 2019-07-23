#pragma once

#include "algo/registration/i_registration.h"
#include "algo/surface_mesh/mesh_definition.h"
#include "algo/registration/deformation_graph/deformation_graph.h"

namespace Registration {

class RigidBeforeNonRigidDeformation
{
public:
	RigidDeformation rigid_deformation;
	DeformationGraph non_rigid_deformation;
	bool is_rigid_deformation;
public:
	RigidBeforeNonRigidDeformation invertDeformation() const;
	RigidBeforeNonRigidDeformation();
};


}