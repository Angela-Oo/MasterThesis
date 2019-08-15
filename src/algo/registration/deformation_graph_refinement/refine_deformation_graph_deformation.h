#pragma once

#include "algo/registration/interface/i_registration.h"
#include "mesh/mesh_definition.h"
#include "algo/triangulation/hierarchical_mesh.h"

namespace Registration {

template<typename NonRigidDeformation>
class RefineDeformationGraphDeformation
{
public:
	using Deformation = NonRigidDeformation;
public:
	NonRigidDeformation non_rigid_deformation;
	HierarchicalMesh hierarchical_mesh;
public:
	RefineDeformationGraphDeformation<NonRigidDeformation> invertDeformation() const;
	RefineDeformationGraphDeformation();
	RefineDeformationGraphDeformation(const RefineDeformationGraphDeformation<NonRigidDeformation> & other);
	RefineDeformationGraphDeformation<NonRigidDeformation> & operator=(const RefineDeformationGraphDeformation<NonRigidDeformation> & other);
};

template<typename NonRigidDeformation>
RefineDeformationGraphDeformation<NonRigidDeformation> RefineDeformationGraphDeformation<NonRigidDeformation>::invertDeformation() const
{
	RefineDeformationGraphDeformation inverted_deformation;
	inverted_deformation.non_rigid_deformation = non_rigid_deformation.invertDeformation();
	return inverted_deformation;
}

template<typename NonRigidDeformation>
RefineDeformationGraphDeformation<NonRigidDeformation>::RefineDeformationGraphDeformation()
	: is_rigid_deformation(true)
{}


template<typename NonRigidDeformation>
RefineDeformationGraphDeformation<NonRigidDeformation>::RefineDeformationGraphDeformation(const RefineDeformationGraphDeformation<NonRigidDeformation> & other)
	: non_rigid_deformation(other.non_rigid_deformation)
{}

template<typename NonRigidDeformation>
RefineDeformationGraphDeformation<NonRigidDeformation> & RefineDeformationGraphDeformation<NonRigidDeformation>::operator=(const RefineDeformationGraphDeformation<NonRigidDeformation> & other)
{
	if (&other == this)
		return *this;
	non_rigid_deformation = other.non_rigid_deformation;
	return *this;
}

}