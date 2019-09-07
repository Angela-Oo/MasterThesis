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
	RefineDeformationGraphDeformation() = default;
	RefineDeformationGraphDeformation(HierarchicalMesh mesh);
	RefineDeformationGraphDeformation(const RefineDeformationGraphDeformation<NonRigidDeformation> & other) = default;
	RefineDeformationGraphDeformation<NonRigidDeformation> & operator=(const RefineDeformationGraphDeformation<NonRigidDeformation> & other) = default;
	RefineDeformationGraphDeformation(RefineDeformationGraphDeformation<NonRigidDeformation> && other) = default;
	RefineDeformationGraphDeformation<NonRigidDeformation> & operator=(RefineDeformationGraphDeformation<NonRigidDeformation> && other) = default;
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
RefineDeformationGraphDeformation<NonRigidDeformation>::RefineDeformationGraphDeformation(HierarchicalMesh mesh)
	: hierarchical_mesh(std::move(mesh))
{}

}