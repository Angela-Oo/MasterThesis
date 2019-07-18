#include "stdafx.h"

#include "arap.h"
#include "arap_cost_functions.h"
#include "algo/ceres_iteration_logger.h"
#include "boost/graph/adjacency_list.hpp"
#include "boost/graph/connected_components.hpp"
#include "algo/mesh_simplification/mesh_simplification.h"
#include "algo/registration/ceres_residual_evaluation.h"
#include <random>

namespace ARAP
{


bool AsRigidAsPossible::random_bool_with_prob(double prob)  // probability between 0.0 and 1.0
{
	std::bernoulli_distribution d(prob);
	return d(_rand_engine);
}

const SurfaceMesh & AsRigidAsPossible::getSource()
{
	return _src;
}

const SurfaceMesh & AsRigidAsPossible::getTarget()
{
	return _dst;
}

SurfaceMesh AsRigidAsPossible::getDeformedPoints()
{
	return _deformed_mesh->deformPoints();
}

SurfaceMesh AsRigidAsPossible::getInverseDeformedPoints()
{
	auto inverse_deformation = invertDeformationGraph(_deformation_graph);
	DG::DeformedMesh deformed(_dst, inverse_deformation, _registration_options.dg_options.number_of_interpolation_neighbors);
	return deformed.deformPoints();
}

const DG::DeformationGraph & AsRigidAsPossible::getDeformationGraph()
{
	return _deformation_graph;
}

SurfaceMesh AsRigidAsPossible::getDeformationGraphMesh()
{
	return deformationGraphToSurfaceMesh(_deformation_graph, _registration_options.evaluate_residuals);
}

std::vector<Point> AsRigidAsPossible::getFixedPostions()
{
	std::vector<Point> positions;
	for (auto & v : _fixed_positions) {
		positions.push_back(_dst.point(v));
	}
	return positions;
}

ResidualIds AsRigidAsPossible::addPointToPointCostForNode(ceres::Problem &problem, vertex_descriptor v, const Point & target_point)
{
	ResidualIds residual_ids;
	float point_to_point_weighting = 0.1f;
	double weight = _registration_options.fit * point_to_point_weighting;

	auto & global = _deformation_graph._global;

	auto n_w_vector = _deformed_mesh->nearestNodes(v).node_weight_vector;

	if (_registration_options.dg_options.number_of_interpolation_neighbors == 4) {
		if (n_w_vector.size() < 4)
			std::cout << "help nearest node is smaller than expected" << std::endl;
		auto d1 = _deformation_graph.getNode(n_w_vector[0].first);
		auto d2 = _deformation_graph.getNode(n_w_vector[1].first);
		auto d3 = _deformation_graph.getNode(n_w_vector[2].first);
		auto d4 = _deformation_graph.getNode(n_w_vector[3].first);

		auto cost_function = FitStarPointToPointAngleAxisCostFunction::Create(target_point, _deformed_mesh->point(v), global._point,
																			  d1._point, d2._point, d3._point, d4._point,
																			  n_w_vector[0].second, n_w_vector[1].second, n_w_vector[2].second, n_w_vector[3].second);

		auto loss_function = new ceres::ScaledLoss(NULL, weight, ceres::TAKE_OWNERSHIP);
		residual_ids.push_back(problem.AddResidualBlock(cost_function, loss_function,
														global._deformation->d(),
														_deformation_graph.getNode(n_w_vector[0].first)._deformation->d(),
														_deformation_graph.getNode(n_w_vector[1].first)._deformation->d(),
														_deformation_graph.getNode(n_w_vector[2].first)._deformation->d(),
														_deformation_graph.getNode(n_w_vector[3].first)._deformation->d()));
	}
	else if (_registration_options.dg_options.number_of_interpolation_neighbors == 3) {
		auto d1 = _deformation_graph.getNode(n_w_vector[0].first);
		auto d2 = _deformation_graph.getNode(n_w_vector[1].first);
		auto d3 = _deformation_graph.getNode(n_w_vector[2].first);

		auto cost_function = FitStarPointToPointAngleAxisCostFunction::Create(target_point, _deformed_mesh->point(v), global._point,
																			  d1._point, d2._point, d3._point,
																			  n_w_vector[0].second, n_w_vector[1].second, n_w_vector[2].second);

		auto loss_function = new ceres::ScaledLoss(NULL, weight, ceres::TAKE_OWNERSHIP);
		residual_ids.push_back(problem.AddResidualBlock(cost_function, loss_function,
														global._deformation->d(),
														_deformation_graph.getNode(n_w_vector[0].first)._deformation->d(),
														_deformation_graph.getNode(n_w_vector[1].first)._deformation->d(),
														_deformation_graph.getNode(n_w_vector[2].first)._deformation->d()));
	}

	return residual_ids;
}

ResidualIds AsRigidAsPossible::addPointToPlaneCostForNode(ceres::Problem &problem,
														  vertex_descriptor v,
														  const Point & target_point, 
														  const Vector & target_normal)
{
	ResidualIds residual_ids;
	float point_to_plane_weighting = 0.9f;
	double weight = _registration_options.fit * point_to_plane_weighting;

	auto & global = _deformation_graph._global;

	auto n_w_vector = _deformed_mesh->nearestNodes(v).node_weight_vector;

	if (_registration_options.dg_options.number_of_interpolation_neighbors == 4) {
		auto d1 = _deformation_graph.getNode(n_w_vector[0].first);
		auto d2 = _deformation_graph.getNode(n_w_vector[1].first);
		auto d3 = _deformation_graph.getNode(n_w_vector[2].first);
		auto d4 = _deformation_graph.getNode(n_w_vector[3].first);

		auto cost_function = FitStarPointToPlaneAngleAxisCostFunction::Create(target_point, target_normal,
																			  _deformed_mesh->point(v), global._point,
																			  d1._point, d2._point, d3._point, d4._point,
																			  n_w_vector[0].second, n_w_vector[1].second, n_w_vector[2].second, n_w_vector[3].second);

		auto loss_function = new ceres::ScaledLoss(NULL, weight, ceres::TAKE_OWNERSHIP);
		residual_ids.push_back(problem.AddResidualBlock(cost_function, loss_function,
														global._deformation->d(),
														_deformation_graph.getNode(n_w_vector[0].first)._deformation->d(),
														_deformation_graph.getNode(n_w_vector[1].first)._deformation->d(),
														_deformation_graph.getNode(n_w_vector[2].first)._deformation->d(),
														_deformation_graph.getNode(n_w_vector[3].first)._deformation->d()));
	}
	else if (_registration_options.dg_options.number_of_interpolation_neighbors == 3) {
		auto d1 = _deformation_graph.getNode(n_w_vector[0].first);
		auto d2 = _deformation_graph.getNode(n_w_vector[1].first);
		auto d3 = _deformation_graph.getNode(n_w_vector[2].first);

		auto cost_function = FitStarPointToPlaneAngleAxisCostFunction::Create(target_point, target_normal,
																			  _deformed_mesh->point(v), global._point,
																			  d1._point, d2._point, d3._point,
																			  n_w_vector[0].second, n_w_vector[1].second, n_w_vector[2].second);

		auto loss_function = new ceres::ScaledLoss(NULL, weight, ceres::TAKE_OWNERSHIP);
		residual_ids.push_back(problem.AddResidualBlock(cost_function, loss_function,
														global._deformation->d(),
														_deformation_graph.getNode(n_w_vector[0].first)._deformation->d(),
														_deformation_graph.getNode(n_w_vector[1].first)._deformation->d(),
														_deformation_graph.getNode(n_w_vector[2].first)._deformation->d()));
	}

	return residual_ids;
}

VertexResidualIds AsRigidAsPossible::addFitCostWithoutICP(ceres::Problem &problem)
{
	VertexResidualIds residual_ids;

	auto & mesh = _deformation_graph._mesh;
	auto deformations = mesh.property_map<vertex_descriptor, std::shared_ptr<IDeformation>>("v:node").first;

	auto target_normals = _dst.property_map<vertex_descriptor, Vector>("v:normal").first;
	for (auto & v : mesh.vertices())
	{
		auto vertex = _deformation_graph.deformNode(v);
		if (_fixed_positions.empty() || (std::find(_fixed_positions.begin(), _fixed_positions.end(), v) != _fixed_positions.end()))
		{
			ResidualIds point_to_point = addPointToPointCostForNode(problem, v, _dst.point(v));
			residual_ids[v].insert(residual_ids[v].end(), point_to_point.begin(), point_to_point.end());
			ResidualIds point_to_plane = addPointToPlaneCostForNode(problem, v, _dst.point(v), target_normals[v]);
			residual_ids[v].insert(residual_ids[v].end(), point_to_plane.begin(), point_to_plane.end());
		}
	}
	//	std::cout << "used nodes " << i << " / " << mesh.number_of_vertices();
	return residual_ids;
}


VertexResidualIds AsRigidAsPossible::addFitCost(ceres::Problem &problem)
{
	VertexResidualIds residual_ids;

	auto & mesh = _deformation_graph._mesh;
	auto deformations = mesh.property_map<vertex_descriptor, std::shared_ptr<IDeformation>>("v:node").first;

	//auto vertex_used = mesh.property_map<vertex_descriptor, bool>("v:vertex_used").first;
	int i = 0;
	for (auto & v : _deformed_mesh->vertices())
	{
		//vertex_used[v] = false;
		
		bool use_vertex = true;

		if (_registration_options.ignore_deformation_graph_border_vertices)
			use_vertex = !_src.is_border(v, true);

		if(_registration_options.use_vertex_random_probability < 1.)
			use_vertex = random_bool_with_prob(_registration_options.use_vertex_random_probability);

		if(use_vertex) {
			auto deformed_point = _deformed_mesh->deformed_point(v);
			auto deformed_normal = _deformed_mesh->deformed_normal(v);

			auto correspondent_point = _find_correspondence_point->correspondingPoint(deformed_point, deformed_normal);

			if (correspondent_point.first) {		
				vertex_descriptor target_vertex = correspondent_point.second;
				auto target_point = _find_correspondence_point->getPoint(target_vertex);
				auto target_normal = _find_correspondence_point->getNormal(target_vertex);

				ResidualIds point_to_point = addPointToPointCostForNode(problem, v, target_point);
				residual_ids[v].insert(residual_ids[v].end(), point_to_point.begin(), point_to_point.end());
				
				if (target_normal.squared_length() > 0.) {
					ResidualIds point_to_plane = addPointToPlaneCostForNode(problem, v, target_point, target_normal);
					residual_ids[v].insert(residual_ids[v].end(), point_to_plane.begin(), point_to_plane.end());
				}
				else {
					std::cout << "normal is degenerated" << std::endl;
				}

				i++;
				//vertex_used[v] = true;
			}
		}
	}
	
	std::cout << "used corresponding vertices " << i << " / " << _deformed_mesh->number_of_vertices() << " ";
	std::cout << " allowed distance " <<  _find_correspondence_point->median() << " ";
	return residual_ids;
}

EdgeResidualIds AsRigidAsPossible::addAsRigidAsPossibleCost(ceres::Problem &problem)
{
	EdgeResidualIds residual_ids;
	auto & mesh = _deformation_graph._mesh;
	auto deformations = mesh.property_map<vertex_descriptor, std::shared_ptr<IDeformation>>("v:node").first;
	for (auto e : mesh.halfedges())
	{		
		auto target = mesh.target(e);
		auto source = mesh.source(e);

		ceres::CostFunction* cost_function = AsRigidAsPossibleCostFunction::Create(mesh.point(source), mesh.point(target));
		auto loss_function = new ceres::ScaledLoss(NULL, _registration_options.smooth, ceres::TAKE_OWNERSHIP);
		auto residual_id = problem.AddResidualBlock(cost_function, loss_function, 
													deformations[source]->d(), deformations[target]->d());

		auto edge = _deformation_graph._mesh.edge(e);
		residual_ids[edge].push_back(residual_id);
	}

	return residual_ids;
}

VertexResidualIds AsRigidAsPossible::addConfCost(ceres::Problem &problem)
{
	VertexResidualIds residual_ids;
	auto deformations = _deformation_graph._mesh.property_map<vertex_descriptor, std::shared_ptr<IDeformation>>("v:node").first;
	for(auto & v : _deformation_graph._mesh.vertices())
	{
		auto& deformation = deformations[v];
		ceres::CostFunction* cost_function = ConfCostFunction::Create();
		auto loss_function = new ceres::ScaledLoss(NULL, _registration_options.conf, ceres::TAKE_OWNERSHIP);
		residual_ids[v].push_back(problem.AddResidualBlock(cost_function, loss_function, deformation->w()));
	}
	return residual_ids;
}

bool AsRigidAsPossible::solveIteration()
{
	if (!finished()) {
		std::cout << std::endl;
		_solve_iteration++;
		ceres::Solver::Summary summary;
		auto logger = _ceres_logger.CreateCeresIterationLogger(summary);

		ceres::Problem problem;

		VertexResidualIds fit_residual_ids;
		if (_with_icp)
			fit_residual_ids = addFitCost(problem);
		else
			fit_residual_ids = addFitCostWithoutICP(problem);
		EdgeResidualIds arap_residual_ids = addAsRigidAsPossibleCost(problem);
		VertexResidualIds conf_residual_ids = addConfCost(problem);

		ceres::Solve(_options, &problem, &summary);

		// evaluate
		if (_registration_options.evaluate_residuals) {
			evaluateResidual(problem, fit_residual_ids, arap_residual_ids, conf_residual_ids);
		}

		_last_cost = _current_cost;
		_current_cost = summary.final_cost;

		auto scale_factor_tol = 0.0001;// 0.00001;
		if (abs(_current_cost - _last_cost) < scale_factor_tol *(1 + _current_cost) &&
			(_registration_options.smooth > 0.005))// && a_conf > 0.05))
		{
			_registration_options.smooth /= 2.;
			_registration_options.conf /= 2.;
			std::cout << std::endl << "scale factor: smooth " << _registration_options.smooth << " conf " << _registration_options.conf << std::endl;
		}
	}
	return finished();
}

size_t AsRigidAsPossible::currentIteration()
{
	return _solve_iteration;
}

bool AsRigidAsPossible::solve()
{
	while (!finished()) {
		solveIteration();
	}
	return true;
}

bool AsRigidAsPossible::finished()
{
	auto tol = _options.function_tolerance;

	double error = abs(_last_cost - _current_cost);
	bool solved = error < (tol * _current_cost);
	return (_solve_iteration >= _registration_options.max_iterations) || (solved && _solve_iteration > 2);
}

void AsRigidAsPossible::evaluateResidual(ceres::Problem & problem,
										 std::map<vertex_descriptor, ResidualIds> & fit_residual_block_ids,
										 std::map<edge_descriptor, ResidualIds> & arap_residual_block_ids,
										 std::map<vertex_descriptor, ResidualIds> & conf_residual_block_ids)
{
	// smooth
	auto smooth_cost = _deformation_graph._mesh.property_map<edge_descriptor, double>("e:smooth_cost");
	if(smooth_cost.second)
		evaluateResiduals(_deformation_graph._mesh, problem, arap_residual_block_ids, smooth_cost.first, _registration_options.smooth);
	
	// fit
	//auto fit_cost = _deformation_graph._mesh.property_map<vertex_descriptor, double>("v:fit_cost");
	//if(fit_cost.second)
	//	evaluateResiduals(_deformation_graph._mesh, problem, fit_residual_block_ids, fit_cost.first, a_fit);

	//// conf
	//auto conf_cost = _deformation_graph._mesh.property_map<vertex_descriptor, double>("v:conf_cost");
	//if (conf_cost.second)
	//	evaluateResiduals(_deformation_graph._mesh, problem, conf_residual_block_ids, conf_cost.first, a_conf);
}


void AsRigidAsPossible::printCeresOptions()
{
	std::cout << "\nCeres Solver" << std::endl;
	std::cout << "Ceres preconditioner type: " << _options.preconditioner_type << std::endl;
	std::cout << "Ceres linear algebra type: " << _options.sparse_linear_algebra_library_type << std::endl;
	std::cout << "Ceres linear solver type: " << _options.linear_solver_type << std::endl;
}

AsRigidAsPossible::AsRigidAsPossible(const SurfaceMesh& src,
									 const SurfaceMesh& dst,
									 std::vector<vertex_descriptor> fixed_positions,
									 ceres::Solver::Options option,
									 RegistrationOptions registration_options,
									 std::shared_ptr<FileWriter> logger)
	: _src(src)
	, _dst(dst)
	, _options(option)
	, _deformation_graph(src, []() { return std::make_shared<Deformation>(); })
	, _fixed_positions(fixed_positions)
	, _ceres_logger(logger)
	, _registration_options(registration_options)
	, _with_icp(false)

{
	_find_correspondence_point = std::make_unique<FindCorrespondingPoints>(dst, _registration_options.correspondence_max_distance, _registration_options.correspondence_max_angle_deviation);
	_deformed_mesh = std::make_unique<DG::DeformedMesh>(src, _deformation_graph, _registration_options.dg_options.number_of_interpolation_neighbors);
	printCeresOptions();
}


AsRigidAsPossible::AsRigidAsPossible(const SurfaceMesh& src,
									 const SurfaceMesh& dst,
									 ceres::Solver::Options option,
									 RegistrationOptions registration_options,								 
									 std::shared_ptr<FileWriter> logger)
	: _src(src)
	, _dst(dst)
	, _options(option)
	, _ceres_logger(logger)
	, _registration_options(registration_options)
{
	_find_correspondence_point = std::make_unique<FindCorrespondingPoints>(dst, _registration_options.correspondence_max_distance, _registration_options.correspondence_max_angle_deviation);
	auto reduced_mesh = createReducedMesh(src, _registration_options.dg_options.edge_length);
	_deformation_graph = DG::DeformationGraph(reduced_mesh, []() { return std::make_shared<Deformation>(); });
	_deformed_mesh = std::make_unique<DG::DeformedMesh>(src, _deformation_graph, _registration_options.dg_options.number_of_interpolation_neighbors);
	
	_ceres_logger.write("number of deformation graph nodes " + _deformation_graph._mesh.number_of_vertices(), false);
	printCeresOptions();
}

AsRigidAsPossible::AsRigidAsPossible(const SurfaceMesh& src,
									 const SurfaceMesh& dst,
									 const DG::DeformationGraph & deformation_graph,
									 ceres::Solver::Options option,
									 RegistrationOptions registration_options,
									 std::shared_ptr<FileWriter> logger)
	: _src(src)
	, _dst(dst)
	, _options(option)
	, _deformation_graph(deformation_graph)
	, _ceres_logger(logger)
	, _registration_options(registration_options)
{
	_find_correspondence_point = std::make_unique<FindCorrespondingPoints>(dst, _registration_options.correspondence_max_distance, _registration_options.correspondence_max_angle_deviation);

	//_deformation_graph = transformDeformationGraph(deformation_graph);
	_deformed_mesh = std::make_unique<DG::DeformedMesh>(src, _deformation_graph, registration_options.dg_options.number_of_interpolation_neighbors);
	printCeresOptions();
}



}
