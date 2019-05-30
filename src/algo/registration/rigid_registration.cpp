#include "stdafx.h"
#include "rigid_registration.h"
#include "algo/ceres_iteration_logger.h"

#include "rigid_deformation_cost_function.h"

#include "algo/registration/deformation_graph/deformation_graph.h" // set color todo

Matrix RigidDeformation::rotation() const
{
	ml::mat3d r;
	ceres::AngleAxisToRotationMatrix(_r.array, r.getData());
	auto r_t = r.getTranspose(); // why??
	Matrix m(r_t(0, 0), r_t(0, 1), r_t(0, 2), r_t(1, 0), r_t(1, 1), r_t(1, 2), r_t(2, 0), r_t(2, 1), r_t(2, 2));
	return m;
}

Vector RigidDeformation::translation() const
{
	return Vector(_t[0], _t[1], _t[2]);
}

Point RigidDeformation::deformPoint(const Point & point) const
{
	Vector rotated_point = rotation()(point - CGAL::ORIGIN);
	Vector moved_position = translation();
	return CGAL::ORIGIN + rotated_point + moved_position;
}

Vector RigidDeformation::deformNormal(const Vector & normal) const
{
	ml::mat3d r;
	ceres::AngleAxisToRotationMatrix(_r.array, r.getData());
	Matrix rotation(r(0, 0), r(0, 1), r(0, 2), r(1, 0), r(1, 1), r(1, 2), r(2, 0), r(2, 1), r(2, 2));
	Vector rotated_normal = rotation(normal);
	return rotated_normal;
}

RigidDeformation::RigidDeformation()
	: _r(ml::vec3f::origin)
	, _t(ml::vec3f::origin)
{}


SurfaceMesh RigidDeformedMesh::deformPoints()
{
	SurfaceMesh deformed_points = _mesh;
	for (auto & v : _mesh.vertices()) {
		deformed_points.point(v) = _deformation.deformPoint(_mesh.point(v));
	}
	return deformed_points;
}


RigidDeformedMesh::RigidDeformedMesh(const SurfaceMesh & mesh, const RigidDeformation & deformation)
	: _mesh(mesh)
	, _deformation(deformation)
{

}



const SurfaceMesh & RigidRegistration::getSource()
{
	return _source;
}

const SurfaceMesh & RigidRegistration::getTarget()
{
	return _target;
}

SurfaceMesh RigidRegistration::getDeformedPoints()
{
	RigidDeformedMesh deformed(getSource(), _deformation);
	return deformed.deformPoints();
}

SurfaceMesh RigidRegistration::getDeformationGraphMesh()
{
	RigidDeformedMesh deformed(_source, _deformation);
	SurfaceMesh mesh = deformed.deformPoints();
	double k_mean_fit_cost = DG::getMeanFitCost(mesh) * 10.;
	DG::setVertexColorBasedOnFitCost(mesh, k_mean_fit_cost);
	return mesh;
};


ceres::ResidualBlockId RigidRegistration::addPointToPointCost(ceres::Problem &problem, const Point & source_point, const Point & target_position)
{
	float point_to_point_weight = 0.1;
	ceres::CostFunction* cost_function = FitPointToPointAngleAxisCostFunction::Create(target_position, source_point);
	auto loss_function = new ceres::ScaledLoss(NULL, point_to_point_weight, ceres::TAKE_OWNERSHIP);
	return problem.AddResidualBlock(cost_function, loss_function, _deformation.r(), _deformation.t());
}

ceres::ResidualBlockId RigidRegistration::addPointToPlaneCost(ceres::Problem &problem, const Point & source_point, const Vector & target_normal, const Point & target_position)
{
	float point_to_plane_weight = 0.9;
	ceres::CostFunction* cost_function = FitPointToPlaneAngleAxisCostFunction::Create(target_position, source_point, target_normal);
	auto loss_function = new ceres::ScaledLoss(NULL, point_to_plane_weight, ceres::TAKE_OWNERSHIP);
	return problem.AddResidualBlock(cost_function, loss_function, _deformation.r(), _deformation.t());
}


std::map<vertex_descriptor, ResidualIds> RigidRegistration::addFitCost(ceres::Problem &problem)
{
	VertexResidualIds residual_ids;

	int i = 0;
	auto normal = _source.property_map<vertex_descriptor, Direction>("v:normal").first;
	for (auto & v : _source.vertices())
	{
		auto point = _source.point(v);
		auto deformed_point = _deformation.deformPoint(point);
		auto deformed_normal = _deformation.deformNormal(normal[v].vector());
		auto correspondent_point = _find_correspondence_point->correspondingPoint(deformed_point, deformed_normal);
		if (correspondent_point.first) {
			auto target_position = _find_correspondence_point->getPoint(correspondent_point.second);
			auto target_normal = _find_correspondence_point->getNormal(correspondent_point.second);
			residual_ids[v].push_back(addPointToPointCost(problem, point, target_position));
			residual_ids[v].push_back(addPointToPlaneCost(problem, point, target_normal.vector(), target_position));
			i++;
		}
	}
	std::cout << "used " << i << " of " << _source.number_of_vertices() << " deformation graph nodes" << std::endl;
	return residual_ids;
}

std::map<vertex_descriptor, ResidualIds> RigidRegistration::addFitCostSubSet(ceres::Problem &problem)
{
	VertexResidualIds residual_ids;

	int i = 0;
	auto normal = _source.property_map<vertex_descriptor, Direction>("v:normal").first;
	
	int _sub_set_size = 100;
	// select random vertices
	std::vector<vertex_descriptor> vertices;
	for (auto & v : _source.vertices())
	{
		vertices.push_back(v);
	}	
	while (vertices.size() > _sub_set_size) {
		int index = (rand() % vertices.size());
		vertices.erase(vertices.begin() + index);
	}

	// cost function
	for(auto v : vertices) {
		auto point = _source.point(v);
		auto correspondent_point = _find_correspondence_point->correspondingPoint(point, normal[v].vector());

		if (correspondent_point.first) {
			auto target_position = _find_correspondence_point->getPoint(correspondent_point.second);
			auto target_normal = _find_correspondence_point->getNormal(correspondent_point.second);
			residual_ids[v].push_back(addPointToPointCost(problem, point, target_position));
			residual_ids[v].push_back(addPointToPlaneCost(problem, point, target_normal.vector(), target_position));
			i++;
		}
	}
	std::cout << "used " << i << " of " << _source.number_of_vertices() << " deformation graph nodes" << std::endl;
	return residual_ids;
}

std::map<vertex_descriptor, ResidualIds> RigidRegistration::addFitCostWithoutICP(ceres::Problem &problem)
{
	VertexResidualIds residual_ids;
	// asuming points are ordered
	auto normal = _source.property_map<vertex_descriptor, Direction>("v:normal").first;
	for (auto & v : _source.vertices())
	{
		residual_ids[v].push_back(addPointToPointCost(problem, _source.point(v), _target.point(v)));
		residual_ids[v].push_back(addPointToPlaneCost(problem, _source.point(v), normal[v].vector(), _target.point(v)));
	}
	return residual_ids;
}

bool RigidRegistration::solveIteration()
{
	if (!finished()) {
		_solve_iteration++;
		ceres::Solver::Summary summary;
		CeresIterationLoggerGuard logger(summary, _total_time_in_ms, _solve_iteration, _logger);

		ceres::Problem problem;

		VertexResidualIds fit_residual_ids;
		if (_with_icp)
			fit_residual_ids = addFitCost(problem);
		else
			fit_residual_ids = addFitCostWithoutICP(problem);

		ceres::Solve(_options, &problem, &summary);

		// evaluate		
		evaluateResidual(problem, fit_residual_ids);

		_last_cost = _current_cost;
		_current_cost = summary.final_cost;

		_total_time_in_ms += logger.get_time_in_ms();
	}
	return finished();
}


bool RigidRegistration::solve()
{
	while (!finished()) {
		solveIteration();
	}
	return true;
}

bool RigidRegistration::finished()
{
	auto tol = _options.function_tolerance * 0.1;

	double error = abs(_last_cost - _current_cost);
	bool solved = error < (tol * _current_cost);
	return (_solve_iteration >= _max_iterations) || (solved && _solve_iteration > 10);
}

void RigidRegistration::evaluateResidual(ceres::Problem & problem,
										 std::map<vertex_descriptor, ResidualIds> & fit_residual_block_ids)
{
	auto fit_cost = _source.add_property_map<vertex_descriptor, double>("v:fit_cost").first;
	for (auto & r : fit_residual_block_ids) {
		ceres::Problem::EvaluateOptions evaluate_options;
		evaluate_options.residual_blocks = r.second;
		double total_cost = 0.0;
		std::vector<double> residuals;
		problem.Evaluate(evaluate_options, &total_cost, &residuals, nullptr, nullptr);
		fit_cost[r.first] = total_cost;
	}
}


//Mesh RigidRegistration::getInverseDeformedPoints()
//{
//	auto transformed_points = _points_a;
//	auto inverse_transformation = _transformation.getInverse();
//	std::for_each(transformed_points.m_vertices.begin(), transformed_points.m_vertices.end(), [&](Mesh::Vertex & p) { p.position = inverse_transformation * p.position; });
//	return transformed_points;
//}


RigidRegistration::RigidRegistration(const SurfaceMesh & source,
									 const SurfaceMesh & target,
									 ceres::Solver::Options option, 
									 std::shared_ptr<FileWriter> logger)
	: _source(source)
	, _target(target)
	, _options(option)
	, _logger(logger)
{
	_find_correspondence_point = std::make_unique<FindCorrespondingPoints>(_target, 0.5, 45.);
	_rigid_deformed_mesh = std::make_unique<RigidDeformedMesh>(_source, _deformation);
}

