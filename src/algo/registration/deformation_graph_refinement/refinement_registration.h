#pragma once

#include "refinement_deformation.h"
#include "refinement_deform_mesh.h"
#include "refinement_deformation_graph.h"
#include "algo/hierarchical_mesh/generate_hierarchical_mesh.h"
#include "algo/registration/interface/i_registration.h"
#include "mesh/mesh_definition.h"


namespace Registration {

template<typename NonRigidRegistration>
class RefineDeformationGraphRegistration : public INonRigidRegistration
{
public:
	using Deformation = RefineDeformationGraphDeformation<typename NonRigidRegistration::Deformation>;
	using PositionDeformation = typename NonRigidRegistration::PositionDeformation;
	using DeformMesh = typename RefinementDeformMesh<typename NonRigidRegistration::Deformation, typename NonRigidRegistration::DeformMesh>;
private:
	using NonRigidDeformation = typename NonRigidRegistration::Deformation;
private:
	Deformation _deformation;
	std::unique_ptr<NonRigidRegistration> _non_rigid_registration;
	RegistrationOptions _options;
	bool _is_refined;
	int _number_of_refinements;
	unsigned int _current_iteration;
	bool _finished;
public:
	bool finished() override;
	bool solveIteration() override;
	size_t currentIteration() override;
	bool solve() override;
public:
	const SurfaceMesh & getSource() override;
	const SurfaceMesh & getTarget() override;
	SurfaceMesh getDeformedPoints() override;
	SurfaceMesh getInverseDeformedPoints() override;
public:
	SurfaceMesh getDeformationGraphMesh() override;
	const Deformation & getDeformation();
	void setRigidDeformation(const RigidDeformation & rigid_deformation) override;
	bool shouldBeSavedAsImage() override;
public:
	//RefineDeformationGraphRegistration(std::unique_ptr<NonRigidRegistration> non_rigid_registration, HierarchicalMesh hierarchical_mesh);

	RefineDeformationGraphRegistration(const SurfaceMesh& source,
									   const SurfaceMesh& target,
									   ceres::Solver::Options option,
									   const RegistrationOptions & registration_options,
									   std::shared_ptr<FileWriter> logger = nullptr);

	RefineDeformationGraphRegistration(const SurfaceMesh& source,
					                   const SurfaceMesh& target,
					                   const Deformation & deformation_graph,
					                   ceres::Solver::Options option,
					                   const RegistrationOptions & registration_options,
					                   std::shared_ptr<FileWriter> logger = nullptr);

	RefineDeformationGraphRegistration(const SurfaceMesh& source,
									   const SurfaceMesh& target,
									   const SurfaceMesh & previous_mesh,
									   const Deformation & deformation_graph,
									   ceres::Solver::Options option,
									   const RegistrationOptions & registration_options,
									   std::shared_ptr<FileWriter> logger = nullptr);
};


template<typename NonRigidRegistration>
const SurfaceMesh & RefineDeformationGraphRegistration<NonRigidRegistration>::getSource()
{
	return _non_rigid_registration->getSource();
}

template<typename NonRigidRegistration>
const SurfaceMesh & RefineDeformationGraphRegistration<NonRigidRegistration>::getTarget()
{
	return _non_rigid_registration->getTarget();
}

template<typename NonRigidRegistration>
SurfaceMesh RefineDeformationGraphRegistration<NonRigidRegistration>::getDeformedPoints()
{
	return _non_rigid_registration->getDeformedPoints();
}

template<typename NonRigidRegistration>
SurfaceMesh RefineDeformationGraphRegistration<NonRigidRegistration>::getInverseDeformedPoints()
{
	return _non_rigid_registration->getInverseDeformedPoints();
}

template<typename NonRigidRegistration>
SurfaceMesh RefineDeformationGraphRegistration<NonRigidRegistration>::getDeformationGraphMesh()
{
	return _non_rigid_registration->getDeformationGraphMesh();
};

template<typename NonRigidRegistration>
bool RefineDeformationGraphRegistration<NonRigidRegistration>::solveIteration()
{
	bool finished = _non_rigid_registration->finished();
	if (finished == false) {
		_current_iteration++;
		_non_rigid_registration->solveIteration();
	}
	else if(_is_refined == false) {
		_deformation.non_rigid_deformation = _non_rigid_registration->getDeformation();
		_deformation.non_rigid_deformation = refineHierarchicalMesh(_deformation, _options.dg_options.number_of_interpolation_neighbors);
		_non_rigid_registration->setDeformation(_deformation.non_rigid_deformation);
		_number_of_refinements++;
		if(_number_of_refinements > 4)
			_is_refined = true;
	}
	else {
		_finished = true;
	}
	return _finished;
}

template<typename NonRigidRegistration>
size_t RefineDeformationGraphRegistration<NonRigidRegistration>::currentIteration()
{
	if (_current_iteration == 0)
		return _current_iteration;
	else
		return _number_of_refinements + 1;
	//return _current_iteration;
}

template<typename NonRigidRegistration>
bool RefineDeformationGraphRegistration<NonRigidRegistration>::solve()
{
	return _non_rigid_registration->solve();
}

template<typename NonRigidRegistration>
bool RefineDeformationGraphRegistration<NonRigidRegistration>::finished()
{
	return _finished;
}

template<typename NonRigidRegistration>
const RefineDeformationGraphDeformation<typename NonRigidRegistration::Deformation> & RefineDeformationGraphRegistration<NonRigidRegistration>::getDeformation()
{
	_deformation.non_rigid_deformation = _non_rigid_registration->getDeformation();
	return _deformation;
}

template<typename NonRigidRegistration>
void RefineDeformationGraphRegistration<NonRigidRegistration>::setRigidDeformation(const RigidDeformation & rigid_deformations)
{
	_non_rigid_registration->setRigidDeformation(rigid_deformations);
}

template<typename NonRigidRegistration>
bool RefineDeformationGraphRegistration<NonRigidRegistration>::shouldBeSavedAsImage()
{
	//bool save = _non_rigid_registration->shouldBeSavedAsImage();
	//bool registration_finished = _non_rigid_registration->finished();
	//return (save && registration_finished);
	return _non_rigid_registration->shouldBeSavedAsImage();
}

//template<typename NonRigidRegistration>
//RefineDeformationGraphRegistration<NonRigidRegistration>::RefineDeformationGraphRegistration(std::unique_ptr<NonRigidRegistration> non_rigid_registration, HierarchicalMesh hierarchical_mesh)
//	: _non_rigid_registration(std::move(non_rigid_registration))
//	, _is_refined(false)
//	, _finished(false)
//	, _number_of_refinements(0)
//	, _current_iteration(0)
//	, _deformation(std::move(hierarchical_mesh))
//{
//}


template<typename NonRigidRegistration>
RefineDeformationGraphRegistration<NonRigidRegistration>::RefineDeformationGraphRegistration(const SurfaceMesh& source,
														                                     const SurfaceMesh& target,
														                                     ceres::Solver::Options ceres_option,
														                                     const RegistrationOptions & options,
														                                     std::shared_ptr<FileWriter> logger = nullptr)
	: _is_refined(false)
	, _finished(false)
	, _number_of_refinements(0)
	, _current_iteration(0)
	, _options(options)
{
	auto hierarchical_mesh = generateHierarchicalMesh(source, options.dg_options.edge_length, options.dg_options.number_of_interpolation_neighbors);
	auto global = createGlobalDeformation<typename NonRigidRegistration::PositionDeformation>(source);
	auto deformation_graph = createDeformationGraphFromMesh<typename NonRigidRegistration::PositionDeformation>(hierarchical_mesh.getInitMesh(), global);

	_non_rigid_registration = std::make_unique<NonRigidRegistration>(source, target, deformation_graph, ceres_option, options, logger);
	_deformation.hierarchical_mesh = hierarchical_mesh;
}

template<typename NonRigidRegistration>
RefineDeformationGraphRegistration<NonRigidRegistration>::RefineDeformationGraphRegistration(const SurfaceMesh& source,
								                                                             const SurfaceMesh& target,
								                                                             const RefineDeformationGraphDeformation<typename NonRigidRegistration::Deformation> & deformation,
								                                                             ceres::Solver::Options ceres_option,
								                                                             const RegistrationOptions & options,
								                                                             std::shared_ptr<FileWriter> logger = nullptr)
	: _is_refined(false)
	, _finished(false)
	, _number_of_refinements(0)
	, _current_iteration(0)
	, _deformation(deformation)
	, _options(options)
{
	_non_rigid_registration = std::make_unique<NonRigidRegistration>(source, target, _deformation.non_rigid_deformation, ceres_option, options, logger);
};


template<typename NonRigidRegistration>
RefineDeformationGraphRegistration<NonRigidRegistration>::RefineDeformationGraphRegistration(const SurfaceMesh& source,
																							 const SurfaceMesh& target,
																							 const SurfaceMesh& previouse_mesh,
																							 const RefineDeformationGraphDeformation<typename NonRigidRegistration::Deformation> & deformation,
																							 ceres::Solver::Options ceres_option,
																							 const RegistrationOptions & options,
																							 std::shared_ptr<FileWriter> logger = nullptr)
	: RefineDeformationGraphRegistration(source, target, deformation, ceres_option, options, logger)
{ };

}

