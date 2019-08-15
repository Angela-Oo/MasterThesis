#pragma once

#include "algo/registration/interface/i_registration.h"
#include "algo/hierarchical_mesh/hierarchical_mesh.h"
#include "mesh/mesh_definition.h"

namespace Registration {

template<typename NonRigidDeformation>
class RefineDeformationGraphDeformation
{
private:
	using Deformation = NonRigidDeformation;
public:
	NonRigidDeformation non_rigid_deformation;
	HierarchicalMesh hierarchical_mesh;
public:
	RigidDeformation getRigidDeformation() const;
	RefineDeformationGraphDeformation<NonRigidDeformation> invertDeformation() const;
public:	
	RefineDeformationGraphDeformation();
	RefineDeformationGraphDeformation(HierarchicalMesh mesh);
	RefineDeformationGraphDeformation(const NonRigidDeformation & non_rigid_deformation);
	RefineDeformationGraphDeformation(const RefineDeformationGraphDeformation<NonRigidDeformation> & other);
	RefineDeformationGraphDeformation<NonRigidDeformation> & operator=(const RefineDeformationGraphDeformation<NonRigidDeformation> & other);
};

template<typename NonRigidDeformation>
RigidDeformation RefineDeformationGraphDeformation<NonRigidDeformation>::getRigidDeformation() const
{
	return non_rigid_deformation.getRigidDeformation();
}


template<typename NonRigidDeformation>
RefineDeformationGraphDeformation<NonRigidDeformation> RefineDeformationGraphDeformation<NonRigidDeformation>::invertDeformation() const
{
	RefineDeformationGraphDeformation<NonRigidDeformation> inverted_deformation;
	inverted_deformation.non_rigid_deformation = non_rigid_deformation.invertDeformation();
	return inverted_deformation;
}

template<typename NonRigidDeformation>
RefineDeformationGraphDeformation<NonRigidDeformation>::RefineDeformationGraphDeformation()
{}

template<typename NonRigidDeformation>
RefineDeformationGraphDeformation<NonRigidDeformation>::RefineDeformationGraphDeformation(HierarchicalMesh mesh)
	: hierarchical_mesh(std::move(mesh))
{}

template<typename NonRigidDeformation>
RefineDeformationGraphDeformation<NonRigidDeformation>::RefineDeformationGraphDeformation(const NonRigidDeformation & non_rigid_deformation)
	: non_rigid_deformation(non_rigid_deformation)
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