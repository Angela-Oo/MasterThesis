#include "rigid_before_non_rigid_deformation.h"

namespace Registration {


RigidBeforeNonRigidDeformation RigidBeforeNonRigidDeformation::invertDeformation() const
{
	RigidBeforeNonRigidDeformation inverted_deformation;
	inverted_deformation.is_rigid_deformation = is_rigid_deformation;
	inverted_deformation.rigid_deformation = rigid_deformation.invertDeformation();
	inverted_deformation.non_rigid_deformation = non_rigid_deformation.invertDeformation();
	return inverted_deformation;
}

RigidBeforeNonRigidDeformation::RigidBeforeNonRigidDeformation()
	: is_rigid_deformation(true)
{}

}
