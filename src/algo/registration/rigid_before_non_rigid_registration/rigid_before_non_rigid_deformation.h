#pragma once

#include "algo/registration/interface/i_registration.h"
#include "mesh/mesh_definition.h"

namespace Registration {

template<typename NonRigidDeformation>
class RigidBeforeNonRigidDeformation
{
public:
	RigidDeformation rigid_deformation;
	NonRigidDeformation non_rigid_deformation;
	bool is_rigid_deformation;
public:
	RigidBeforeNonRigidDeformation<NonRigidDeformation> invertDeformation() const;
	RigidBeforeNonRigidDeformation();
	RigidBeforeNonRigidDeformation(const RigidBeforeNonRigidDeformation<NonRigidDeformation> & other);
	RigidBeforeNonRigidDeformation<NonRigidDeformation> & operator=(const RigidBeforeNonRigidDeformation<NonRigidDeformation> & other);
};

template<typename NonRigidDeformation>
RigidBeforeNonRigidDeformation<NonRigidDeformation> RigidBeforeNonRigidDeformation<NonRigidDeformation>::invertDeformation() const
{
	RigidBeforeNonRigidDeformation inverted_deformation;
	inverted_deformation.is_rigid_deformation = is_rigid_deformation;
	inverted_deformation.rigid_deformation = rigid_deformation.invertDeformation();
	inverted_deformation.non_rigid_deformation = non_rigid_deformation.invertDeformation();
	return inverted_deformation;
}

template<typename NonRigidDeformation>
RigidBeforeNonRigidDeformation<NonRigidDeformation>::RigidBeforeNonRigidDeformation()
	: is_rigid_deformation(true)
{}


template<typename NonRigidDeformation>
RigidBeforeNonRigidDeformation<NonRigidDeformation>::RigidBeforeNonRigidDeformation(const RigidBeforeNonRigidDeformation<NonRigidDeformation> & other)
	: is_rigid_deformation(other.is_rigid_deformation)
	, rigid_deformation(other.rigid_deformation)
	, non_rigid_deformation(other.non_rigid_deformation)
{}

template<typename NonRigidDeformation>
RigidBeforeNonRigidDeformation<NonRigidDeformation> & RigidBeforeNonRigidDeformation<NonRigidDeformation>::operator=(const RigidBeforeNonRigidDeformation<NonRigidDeformation> & other)
{
	if (&other == this)
		return *this;
	is_rigid_deformation = other.is_rigid_deformation;
	rigid_deformation = other.rigid_deformation;
	non_rigid_deformation = other.non_rigid_deformation;
	return *this;
}

}