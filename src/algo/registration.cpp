#include "registration.h"

ceres::Solver::Options ceresOption() {
	ceres::Solver::Options options;
	options.sparse_linear_algebra_library_type = ceres::EIGEN_SPARSE;
	options.minimizer_type = ceres::MinimizerType::TRUST_REGION;
	options.trust_region_strategy_type = ceres::TrustRegionStrategyType::LEVENBERG_MARQUARDT;
	options.line_search_direction_type = ceres::LineSearchDirectionType::LBFGS;
	options.linear_solver_type = ceres::LinearSolverType::SPARSE_NORMAL_CHOLESKY; //ceres::LinearSolverType::CGNR
	options.preconditioner_type = ceres::PreconditionerType::JACOBI;// SCHUR_JACOBI;
	options.max_num_iterations = 50;
	options.logging_type = ceres::LoggingType::SILENT;
	options.minimizer_progress_to_stdout = false;
	return options;
}

//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------

bool RigidRegistration::solve()
{
	if (!_icp_nn->finished()) {
		_transformation = _icp_nn->solveIteration();
		return true;
	}
	return false;
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

RigidRegistration::RigidRegistration(const Mesh & points_a, const Mesh & points_b, std::shared_ptr<FileWriter> logger)
	: _points_a(points_a)
	, _points_b(points_b)
	, _transformation(ml::mat4f::identity())
	, _logger(logger)
{
	std::vector<ml::vec3f> positions_a;
	for (auto & p : _points_a.getVertices())
		positions_a.push_back(p.position);
	
	std::vector<ml::vec3f> positions_b;
	for (auto & p : _points_b.getVertices())
		positions_b.push_back(p.position);
	_icp_nn = std::make_unique<ICPNN>(positions_a, positions_b, ceresOption(), logger);
}

//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------

bool NonRigidRegistration::solve()
{
	if (!_embedded_deformation->finished()) {
		_embedded_deformation->solveIteration();
		
		return true;
	}
	return false;
}

Mesh NonRigidRegistration::getPointsA()
{
	return _embedded_deformation->getDeformedPoints();
}

Mesh NonRigidRegistration::getPointsB()
{
	return _embedded_deformation->getTarget();
}

std::vector<ml::vec3f> NonRigidRegistration::getPointsDeformationGraph()
{
	return _embedded_deformation->getEmeddedDeformationGraph().getDeformationGraph();
}

std::pair<std::vector<ml::vec3f>, std::vector<ml::vec3f>> NonRigidRegistration::getDeformationGraph()
{
	return _embedded_deformation->getDeformationGraph();
}

std::vector<ml::vec3f> NonRigidRegistration::getFixedPositions()
{
	return _embedded_deformation->getFixedPostions();
}

NonRigidRegistration::NonRigidRegistration(const Mesh & points_a, const Mesh & points_b, std::vector<int> fixed_positions, unsigned int number_of_deformation_nodes, std::shared_ptr<FileWriter> logger)
{
	//options.linear_solver_type = ceres::LinearSolverType::SPARSE_NORMAL_CHOLESKY; //ceres::LinearSolverType::CGNR
	//options.preconditioner_type = ceres::PreconditionerType::JACOBI;// SCHUR_JACOBI;

	//_embedded_deformation = std::make_unique<ED::EmbeddedDeformation>(_points_a, _points_b, ceresOption(), number_of_deformation_nodes, logger);
	_embedded_deformation = std::make_unique<ED::EmbeddedDeformationWithoutICP>(points_a, points_b, fixed_positions, ceresOption(), logger);
}


//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------

bool ARAPNonRigidRegistration::solve()
{
	if (!_as_rigid_as_possible->finished()) {
		_as_rigid_as_possible->solveIteration();
		return true;
	}
	return false;
}

Mesh ARAPNonRigidRegistration::getPointsA()
{
	return _as_rigid_as_possible->getDeformedPoints();
}

Mesh ARAPNonRigidRegistration::getPointsB()
{
	return _as_rigid_as_possible->getTarget();
}

std::vector<ml::vec3f> ARAPNonRigidRegistration::getPointsDeformationGraph()
{
	return _as_rigid_as_possible->getARAPDeformationGraph().getDeformationGraph();
}

std::pair<std::vector<ml::vec3f>, std::vector<ml::vec3f>> ARAPNonRigidRegistration::getDeformationGraph()
{
	return _as_rigid_as_possible->getDeformationGraph();
}

std::vector<ml::vec3f> ARAPNonRigidRegistration::getFixedPositions()
{
	return _as_rigid_as_possible->getFixedPostions();
}

ARAPNonRigidRegistration::ARAPNonRigidRegistration(const Mesh & points_a, const Mesh & points_b, std::vector<int> fixed_positions, unsigned int number_of_deformation_nodes, std::shared_ptr<FileWriter> logger)
{
	_as_rigid_as_possible = std::make_unique<AsRigidAsPossibleWithoutICP>(points_a, points_b, fixed_positions, ceresOption(), logger);
}


//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------

bool NonRigidRegistrationFrames::solve() 
{
	if (_current >= _meshes.size())
		throw std::exception("not enouth meshes");
	if (!_embedded_deformation) {
		//_embedded_deformation = std::make_unique<EmbeddedDeformation>(_meshes[0], _meshes[_current], /*_deformation_graphs[_current - 1],*/ options, _number_of_deformation_nodes);
		_embedded_deformation = std::make_unique<ED::EmbeddedDeformation>(_meshes[0], _meshes[_current], _deformation_graphs[_current - 1], ceresOption(), _number_of_deformation_nodes);
	}
	if (_embedded_deformation && !_embedded_deformation->finished()) {
		_embedded_deformation->solveIteration();
		_deformed_meshes[_current] = _embedded_deformation->getInverseDeformedPoints();
		_deformation_graphs[_current] = _embedded_deformation->getEmbeddedDeformationGraph();
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
