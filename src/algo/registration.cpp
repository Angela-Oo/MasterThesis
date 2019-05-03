#include "registration.h"

bool RigidRegistration::solve()
{
	std::vector<ml::vec3f> positions_a;
	for (auto & p : _points_a.getVertices())
		positions_a.push_back(p.position);
	
	std::vector<ml::vec3f> positions_b;
	for (auto & p : _points_b.getVertices())
		positions_b.push_back(p.position);
	_transformation = iterative_closest_points(positions_a, positions_b);
	return true;
}


void RigidRegistration::icp_calc_nn_in_cost_function()
{
	if (!_icp_nn) {
		ceres::Solver::Options options;
		options.sparse_linear_algebra_library_type = ceres::EIGEN_SPARSE;
		options.minimizer_type = ceres::MinimizerType::TRUST_REGION;
		options.trust_region_strategy_type = ceres::TrustRegionStrategyType::LEVENBERG_MARQUARDT;
		options.line_search_direction_type = ceres::LineSearchDirectionType::LBFGS;
		options.linear_solver_type = ceres::LinearSolverType::SPARSE_NORMAL_CHOLESKY;
		options.preconditioner_type = ceres::PreconditionerType::JACOBI;// SCHUR_JACOBI;
		options.max_num_iterations = 50;
		options.logging_type = ceres::LoggingType::SILENT;
		options.minimizer_progress_to_stdout = false;
		_icp_nn = std::make_unique<ICP>(_points_a, _points_b, options);

		//if (!_icp_nn->finished())
			//_transformation = _icp_nn->solveIteration();
		_transformation = _icp_nn->solve();
	}
}

Mesh RigidRegistration::getPointsA()
{
	auto transformed_points = _points_a;
	std::for_each(transformed_points.m_vertices.begin(), transformed_points.m_vertices.end(), [&](Mesh::Vertex & p) { p.position = _transformation * p.position; });
	return transformed_points;
}

Mesh RigidRegistration::getPointsB()
{
	return _points_b;
}

std::vector<ml::vec3f> RigidRegistration::getPointsDeformationGraph()
{
	return std::vector<ml::vec3f>();
}

RigidRegistration::RigidRegistration(const Mesh & points_a, const Mesh & points_b)
	: _points_a(points_a)
	, _points_b(points_b)
	, _transformation(ml::mat4f::identity())
{}

//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------

bool NonRigidRegistration::solve()
{
	//AsRigidAsPossible arap(_points_a, _points_b, options);
	//_points_b = arap.solve();
	if (!(_embedded_deformation || _as_rigid_as_possible)) {
		ceres::Solver::Options options;
		options.sparse_linear_algebra_library_type = ceres::EIGEN_SPARSE;
		options.minimizer_type = ceres::MinimizerType::TRUST_REGION;
		options.trust_region_strategy_type = ceres::TrustRegionStrategyType::LEVENBERG_MARQUARDT;
		options.line_search_direction_type = ceres::LineSearchDirectionType::LBFGS;
		options.linear_solver_type = ceres::LinearSolverType::SPARSE_NORMAL_CHOLESKY; //ceres::LinearSolverType::CGNR
		options.preconditioner_type = ceres::PreconditionerType::JACOBI;// SCHUR_JACOBI;
		options.max_num_iterations = 100;
		options.logging_type = ceres::LoggingType::SILENT;
		options.minimizer_progress_to_stdout = false;

		_embedded_deformation = std::make_unique<ED::EmbeddedDeformation>(_points_a, _points_b, options, _number_of_deformation_nodes, _logger);
		//_as_rigid_as_possible = std::make_unique<AsRigidAsPossible>(_points_a, _points_b, options, _number_of_deformation_nodes, _logger);
	}
	if (_embedded_deformation && !_embedded_deformation->finished()) {
		_embedded_deformation->solveIteration();
		_points_a = _embedded_deformation->getDeformedPoints();
		return true;
	}
	if (_as_rigid_as_possible && !_as_rigid_as_possible->finished()) {
		_as_rigid_as_possible->solveIteration();
		_points_a = _as_rigid_as_possible->getDeformedPoints();
		return true;
	}
	return false;
}


Mesh NonRigidRegistration::getPointsA()
{
	return _points_a;
}

Mesh NonRigidRegistration::getPointsB()
{
	return _points_b;
}

std::vector<ml::vec3f> NonRigidRegistration::getPointsDeformationGraph()
{
	if (_embedded_deformation)
		return _embedded_deformation->getDeformationGraph().getDeformationGraph();
	else if (_as_rigid_as_possible)
		return _as_rigid_as_possible->getDeformationGraph().getDeformationGraph();
	else
		return std::vector<ml::vec3f>();
}

NonRigidRegistration::NonRigidRegistration()
	: _transformation()
{
	for (int i = 0; i < 50; i++) {
		float x = 0.01 *static_cast<float>(i);
		ml::TriMeshf::Vertex vertex;
		vertex.position = { x,0.,0. };
		_points_a.m_vertices.push_back(vertex);
	}

	_points_b = _points_a;
	for (int i = 25; i < 50; i++) {
		float z = 0.005 *static_cast<double>(i - 24);
		_points_b.m_vertices[i].position.y = z;
	}
}

NonRigidRegistration::NonRigidRegistration(const Mesh & points_a, const Mesh & points_b, unsigned int number_of_deformation_nodes, std::shared_ptr<FileWriter> logger)
	: _points_a(points_a)
	, _points_b(points_b)
	, _number_of_deformation_nodes(number_of_deformation_nodes)
	, _logger(logger)
{}




//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------

bool NonRigidRegistrationFrames::solve() 
{
	if (_current >= _meshes.size())
		throw std::exception("not enouth meshes");
	if (!_embedded_deformation) {
		ceres::Solver::Options options;
		options.sparse_linear_algebra_library_type = ceres::EIGEN_SPARSE;
		options.minimizer_type = ceres::MinimizerType::TRUST_REGION;
		options.trust_region_strategy_type = ceres::TrustRegionStrategyType::LEVENBERG_MARQUARDT;
		options.line_search_direction_type = ceres::LineSearchDirectionType::LBFGS;
		options.linear_solver_type = ceres::LinearSolverType::SPARSE_NORMAL_CHOLESKY; //ceres::LinearSolverType::CGNR
		options.preconditioner_type = ceres::PreconditionerType::JACOBI;// SCHUR_JACOBI;
		options.max_num_iterations = 100;
		options.logging_type = ceres::LoggingType::SILENT;
		options.minimizer_progress_to_stdout = false;

		//_embedded_deformation = std::make_unique<EmbeddedDeformation>(_meshes[0], _meshes[_current], /*_deformation_graphs[_current - 1],*/ options, _number_of_deformation_nodes);
		_embedded_deformation = std::make_unique<ED::EmbeddedDeformation>(_meshes[0], _meshes[_current], _deformation_graphs[_current - 1], options, _number_of_deformation_nodes);
		//_embedded_deformation = std::make_unique<EmbeddedDeformation>(_meshes[_current], _meshes[0], options, _number_of_deformation_nodes);
	}
	if (_embedded_deformation && !_embedded_deformation->finished()) {
		_embedded_deformation->solveIteration();
		_deformed_meshes[_current] = _embedded_deformation->getInverseDeformedPoints();
		_deformation_graphs[_current] = _embedded_deformation->getDeformationGraph();
		return true;
	}
	else {
		if (_current < _meshes.size() - 1) {
			std::cout << std::endl << "frame " << _current << " solved" << std::endl;
			_current++;			
			_embedded_deformation.reset();
			return true;
		}
		return false;
	}
}

bool NonRigidRegistrationFrames::finished()
{
	return (_current >= _meshes.size() - 1);
}

Mesh NonRigidRegistrationFrames::getMesh(int frame) 
{
	return _meshes[frame];
}

Mesh NonRigidRegistrationFrames::getDeformedMesh(int frame)
{
	return _deformed_meshes[frame];
}

size_t NonRigidRegistrationFrames::getCurrent()
{
	return _current;
}

DeformationGraph<ED::Graph, ED::Node> NonRigidRegistrationFrames::getDeformationGraph(int frame)
{
	return _deformation_graphs[frame];
}

NonRigidRegistrationFrames::NonRigidRegistrationFrames()
	: _current(1)
{
}

NonRigidRegistrationFrames::NonRigidRegistrationFrames(const std::vector<Mesh> & meshes, unsigned int number_of_deformation_nodes)
	: _meshes(meshes)
	, _number_of_deformation_nodes(number_of_deformation_nodes)
	, _current(1)	
{
	_deformation_graphs.resize(_meshes.size());
	_deformed_meshes.resize(_meshes.size());
	_deformation_graphs[0] = DeformationGraph<ED::Graph, ED::Node>(_meshes[0], _number_of_deformation_nodes);
	_deformed_meshes[0] = _meshes[0];
}
