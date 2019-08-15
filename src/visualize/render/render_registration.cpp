#include "render_registration.h"

#include "algo/registration_evaluation/evaluate_registration.h"
#include "render_deformation_graph.h"

std::string renderMeshModeToString(Render render_type)
{
	if (render_type == Render::ALL)
		return "ALL";
	else if (render_type == Render::DEFORMATION)
		return "DEFORMATION";
	else if (render_type == Render::TARGET)
		return "TARGET";
	else if (render_type == Render::ONLY_DEFORMATION_GRAPH)
		return "ONLY_DEFORMATION_GRAPH";
	else if(render_type == Render::NONE)
		return "NONE";

	return "";
}


void RenderRegistration::render(ml::Cameraf& camera)
{
	_mesh_renderer->render(camera);
	_point_renderer->render(camera);
}

Render RenderRegistration::nextRenderMeshMode()
{
	if (_render_mesh == Render::ALL)
		_render_mesh = Render::DEFORMATION;
	else if (_render_mesh == Render::DEFORMATION)
		_render_mesh = Render::TARGET;
	else if (_render_mesh == Render::TARGET)
		_render_mesh = Render::ONLY_DEFORMATION_GRAPH;
	else if (_render_mesh == Render::ONLY_DEFORMATION_GRAPH)
		_render_mesh = Render::NONE;
	else
		_render_mesh = Render::ALL;
		
	return _render_mesh;
}

void RenderRegistration::saveCurrentWindowAsImage(std::string folder, std::string filename)
{
	try {
		_mesh_renderer->saveCurrentWindowAsImage(folder, filename);
	}
	catch (...) {
		std::cout << "could not save image " << filename << std::endl;
	}
}

void RenderRegistration::renderMesh(std::string id, SurfaceMesh & mesh, ml::RGBColor color)
{
	_mesh_renderer->insertMesh(id, mesh, color);
}

void RenderRegistration::renderCurrentFrame(std::shared_ptr<IMeshReader> mesh_reader, bool visible)
{
	if (visible)
	{
		bool override_mesh = _last_rendered_current_frame != _current_frame;
		if (_render_mesh == Render::ONLY_DEFORMATION_GRAPH) {
			_mesh_renderer->removeMesh("mesh");
			_point_renderer->insertMesh("mesh", mesh_reader->getMesh(_current_frame), 0.001f, false, override_mesh);
			_point_renderer->insertPoints("mesh_p", mesh_reader->getMesh(_current_frame), ml::RGBColor::Green, 0.005f, override_mesh);
		}
		else {
			_point_renderer->removePoints("mesh");
			_point_renderer->removePoints("mesh_p");
			_mesh_renderer->insertMesh("mesh", mesh_reader->getMesh(_current_frame), ml::RGBColor::White.toVec4f(), override_mesh);
		}
		_last_rendered_current_frame = _current_frame;
	}
	else {
		_mesh_renderer->removeMesh("mesh");
		_point_renderer->removePoints("mesh");
	}
}

void RenderRegistration::renderSelectedFrames(std::shared_ptr<IMeshReader> mesh_reader, std::vector<unsigned int> selected_frames)
{
		// render mesh if selected
	if (!selected_frames.empty())
	{
		_mesh_renderer->insertMesh("selected_mesh_a", mesh_reader->getMesh(selected_frames[0]), ml::RGBColor::Cyan.toVec4f());
		if (selected_frames.size() >= 2)
		{
			_mesh_renderer->insertMesh("selected_mesh_b", mesh_reader->getMesh(selected_frames[1]), ml::RGBColor::Green.toVec4f(), false);
		}
	}
	else {
		_mesh_renderer->removeMesh("selected_mesh_a");
		_mesh_renderer->removeMesh("selected_mesh_b");
	}
}

void RenderRegistration::renderReference(std::shared_ptr<IMeshReader> mesh_reader)
{
	// reference
	if (mesh_reader->size() > _current_frame) {
		if (_render_reference_mesh) {
			bool override_mesh = (_current_frame != _last_rendered_reference_frame);
			_mesh_renderer->insertMesh("reference", mesh_reader->getMesh(_current_frame), ml::RGBColor::White.toVec4f(), override_mesh);
			_last_rendered_reference_frame = _current_frame;
		}
		else {
			_mesh_renderer->removeMesh("reference");
		}
	}
}


void RenderRegistration::renderDeformedSourceMesh(const SurfaceMesh & deformed_points, bool override, bool debug_normals)
{
	// render point clouds
	std::string deformed_points_id = "deformed_points_id";
	std::string deformed_mesh_id = "deformed_mesh_id";

	if (_render_mesh == Render::NONE) { // only render deformed mesh as points 
		_point_renderer->insertPoints(deformed_points_id, deformed_points, ml::RGBColor::Cyan, 0.001f, override);
		_mesh_renderer->removeMesh(deformed_mesh_id);
	}
	else if (_render_mesh == Render::DEFORMATION || _render_mesh == Render::ALL) { // render deformed mesh
		_point_renderer->removePoints(deformed_points_id);
		_mesh_renderer->insertMesh(deformed_mesh_id, deformed_points, ml::RGBColor::Cyan.toVec4f(), override);		
	}
	else if (_render_mesh == Render::TARGET || _render_mesh == Render::ONLY_DEFORMATION_GRAPH, override) { // no deformed mesh
		_mesh_renderer->removeMesh(deformed_mesh_id);
		_point_renderer->removePoints(deformed_points_id);
	}

	// debug normals
	std::string deformed_normals_id = "deformed_normals_id";
	if (debug_normals && (_render_mesh == Render::DEFORMATION || _render_mesh == Render::ALL)) {
		_point_renderer->insertMesh(deformed_normals_id, deformed_points, ml::RGBColor::Cyan, 0.001f, true);
	}
	else {
		_point_renderer->removePoints(deformed_normals_id);
	}
}

void RenderRegistration::renderTargetMesh(const SurfaceMesh & target, bool override, bool debug_normals)
{
	std::string target_points_id = "target_points_id";
	std::string target_mesh_id = "target_mesh_id";

	if (_render_mesh == Render::NONE || _render_mesh == Render::DEFORMATION) { // only render target as points
		_point_renderer->insertPoints(target_points_id, target, ml::RGBColor::Green, 0.001f, override);
		_mesh_renderer->removeMesh(target_mesh_id);
	}
	else if (_render_mesh == Render::ALL || _render_mesh == Render::TARGET) { // render target mesh
		_mesh_renderer->insertMesh(target_mesh_id, target, ml::RGBColor::Green.toVec4f(), override);
		_point_renderer->removePoints(target_points_id);
	}
	else if (_render_mesh == Render::ONLY_DEFORMATION_GRAPH) { // no target
		_mesh_renderer->removeMesh(target_mesh_id);
		_point_renderer->removePoints(target_points_id);
	}

	// debug normals
	std::string target_normals_id = "target_normals_id";
	if (debug_normals && (_render_mesh == Render::TARGET || _render_mesh == Render::ALL)) {
		_point_renderer->insertMesh(target_normals_id, target, ml::RGBColor::Green, 0.001f, true);
	}
	else {
		_point_renderer->removePoints(target_normals_id);
	}
}

void RenderRegistration::renderDeformationGraph(SurfaceMesh & deformation_graph, bool debug_normals)
{
	if (_render_deformation_graph) {
		Visualize::setDeformationGraphColor(deformation_graph, true);
		_point_renderer->insertMesh("deformation_graph", deformation_graph, 0.001f, debug_normals, true);
	}
	else {
		_point_renderer->removePoints("deformation_graph");
	}
}

void RenderRegistration::renderRegistration(std::shared_ptr<IRegistration> registration)
{
	bool debug_normals = false;
	bool debug_deformation_graph_normals = false;
	if (registration)
	{
		auto deformed_points = registration->getDeformedPoints();

		renderDeformedSourceMesh(deformed_points, true, debug_normals);
		renderTargetMesh(registration->getTarget(), false, debug_normals);

		auto non_rigid_registration = dynamic_cast<INonRigidRegistration*>(registration.get()); // todo .... maybe external polymorthis

		if (non_rigid_registration) {
			// fixed positions
			std::vector<Point> render_fixed_positions = non_rigid_registration->getFixedPostions();
			if (!render_fixed_positions.empty())
				_point_renderer->insertPoints("frame_fixed_positions", render_fixed_positions, ml::RGBColor::Red, 0.005f);

			// deformation graph
			auto render_dg = non_rigid_registration->getDeformationGraphMesh();
			renderDeformationGraph(render_dg, debug_deformation_graph_normals);
		}
	}
}


void RenderRegistration::renderRegistrationSequence(std::shared_ptr<ISequenceRegistration> sequence_registration)
{
	if (sequence_registration) {
		auto current = sequence_registration->getCurrent();

		if (_reigistration_finished) {
			_mesh_renderer->removeMesh("target");
			_mesh_renderer->removeMesh("deformed");
			_mesh_renderer->removeMesh("inverse_mesh");
			_mesh_renderer->removeMesh("mesh_source");

			if (_render_mesh == Render::NONE) {
				for (int i = 0; i <= current; ++i) {
					std::string key = "mesh_" + std::to_string(i);
					if (!_mesh_renderer->keyExists(key)) {
						auto inverse_deformed_points = sequence_registration->getInverseDeformedMesh(i);
						_mesh_renderer->insertMesh(key, inverse_deformed_points, ml::RGBColor::Cyan.toVec4f(), false);
					}
				}
				_point_renderer->removePoints("deformation_graph");
			}
			else {
				for (int i = 0; i <= current; ++i) {
					std::string key = "mesh_" + std::to_string(i);
					_mesh_renderer->removeMesh(key);
				}

				auto deformed_points = sequence_registration->getDeformedMesh(_current_frame);
				auto target = sequence_registration->getMesh(_current_frame);

				renderDeformedSourceMesh(deformed_points, true);
				renderTargetMesh(target, true);

				// deformation graph
				auto render_dg = sequence_registration->getDeformationGraphMesh(_current_frame);
				renderDeformationGraph(render_dg);

				if (_render_mesh == Render::DEFORMATION) {
					auto inverse_deformed_points = sequence_registration->getInverseDeformedMesh(_current_frame);
					_mesh_renderer->insertMesh("inverse_mesh", inverse_deformed_points, ml::RGBColor::Cyan.toVec4f(), false);

					auto source = sequence_registration->getMesh(0);
					_mesh_renderer->insertMesh("mesh_source", source, ml::RGBColor::Yellow.toVec4f(), false);
				}
			}
		}
		else {
			_reigistration_finished = sequence_registration->finished();

			auto deformed_points = sequence_registration->getDeformedMesh(current);
			auto target = sequence_registration->getMesh(current);

			renderDeformedSourceMesh(deformed_points, true);
			renderTargetMesh(target, true);

			// deformation graph
			auto render_dg = sequence_registration->getDeformationGraphMesh(sequence_registration->getCurrent());
			renderDeformationGraph(render_dg);
		}
	}
}


void RenderRegistration::renderError(std::vector<std::pair<Point, Point>> error_points)
{
	if (_render_error) {
		auto distance_errors = evaluate_distance_error(error_points);
		float average = std::accumulate(distance_errors.begin(), distance_errors.end(), 0.0f) / static_cast<float>(distance_errors.size());
		float max = *std::max_element(distance_errors.begin(), distance_errors.end());

		std::vector<Edge> edges;
		for (int i = 0; i < error_points.size(); ++i) {
			Edge e;
			e.source_point = PointToVec3f(error_points[i].first);
			e.target_point = PointToVec3f(error_points[i].second);
			auto max_cost = distance_errors[distance_errors.size() / 2] * 10.;
			e.cost = std::min(max_cost, dist(e.source_point, e.target_point));
			e.cost /= max_cost;
			edges.push_back(e);
		}
		_point_renderer->insertLine("error", edges, 0.0005f);
	}
	else {
		_point_renderer->removePoints("error");
	}
}

RenderRegistration::RenderRegistration(ml::GraphicsDevice * graphics)
{
	_mesh_renderer = std::make_unique<MeshRenderer>(graphics);
	_point_renderer = std::make_unique<PointsRenderer>(graphics);
	_render_points = true;
	_render_mesh = Render::ALL;
	_render_reference_mesh = true;
	_render_error = false;
	_render_deformation_graph = true;
	_current_frame = 0;
	_last_rendered_current_frame = 0;
	_last_rendered_reference_frame = 0;
	_reigistration_finished = false;
}