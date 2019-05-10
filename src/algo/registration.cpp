#include "registration.h"

ceres::Solver::Options ceresOption() {
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
	return options;
}

//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------

bool RigidRegistration::solve()
{
	while (!_icp_nn->finished()) {
		_transformation = _icp_nn->solveIteration();
	}
	return true;
}

bool RigidRegistration::finished()
{
	return _icp_nn->finished();
}


bool RigidRegistration::solveIteration()
{
	if (!_icp_nn->finished()) {
		_transformation = _icp_nn->solveIteration();
	}
	return finished();
}

const Mesh & RigidRegistration::getSource()
{
	return _points_a;
}

const Mesh & RigidRegistration::getTarget()
{
	return _points_b;
}

Mesh RigidRegistration::getDeformedPoints()
{
	auto transformed_points = _points_a;
	std::for_each(transformed_points.m_vertices.begin(), transformed_points.m_vertices.end(), [&](Mesh::Vertex & p) { p.position = _transformation * p.position; });
	return transformed_points;
}

Mesh RigidRegistration::getInverseDeformedPoints()
{
	auto transformed_points = _points_a;
	auto inverse_transformation = _transformation.getInverse();
	std::for_each(transformed_points.m_vertices.begin(), transformed_points.m_vertices.end(), [&](Mesh::Vertex & p) { p.position = inverse_transformation * p.position; });
	return transformed_points;
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
