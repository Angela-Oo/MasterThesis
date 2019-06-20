#include "render_registration.h"

#include "algo/evaluate_registration.h"

void RenderRegistration::render(ml::Cameraf& camera)
{
	_mesh_renderer->render(camera);
	_point_renderer->render(camera);
}

Render RenderRegistration::nextRenderMeshMode()
{
	if (_render_mesh == Render::NONE)
		_render_mesh = Render::DEFORMATION;
	else if (_render_mesh == Render::DEFORMATION)
		_render_mesh = Render::TARGET;
	else if (_render_mesh == Render::TARGET)
		_render_mesh = Render::ALL;
	else
		_render_mesh = Render::NONE;
	return _render_mesh;
}

void RenderRegistration::saveCurrentWindowAsImage(std::string folder, std::string filename)
{
	_mesh_renderer->saveCurrentWindowAsImage(folder, filename);
}

void RenderRegistration::renderMesh(std::string id, SurfaceMesh & mesh, ml::RGBColor color)
{
	_mesh_renderer->insertMesh(id, mesh, color);
}

void RenderRegistration::renderCurrentFrame(std::shared_ptr<IMeshReader> mesh_reader, unsigned int current_frame, bool visible)
{
	if (visible)
	{
		bool override_mesh = _current_frame != current_frame;
		_mesh_renderer->insertMesh("mesh", mesh_reader->getMesh(current_frame), ml::RGBColor::White.toVec4f(), override_mesh);
		_current_frame = current_frame;
	}
	else {
		_mesh_renderer->removeMesh("mesh");
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

void RenderRegistration::renderReference(std::shared_ptr<IMeshReader> mesh_reader, unsigned int current_frame)
{
	// reference
	if (_render_reference_mesh) {
		bool override_mesh = (current_frame != _current_reference_frame);
		_mesh_renderer->insertMesh("reference", mesh_reader->getMesh(current_frame), ml::RGBColor::White.toVec4f(), override_mesh);
		_current_reference_frame = current_frame;
	}
	else {
		_mesh_renderer->removeMesh("reference");
	}
}

void RenderRegistration::renderRegistration(std::shared_ptr<IRegistration> registration)
{
	if (registration)
	{
		if (_render_points || _render_mesh != Render::NONE) {
			auto deformed_points = registration->getDeformedPoints();

			// render point clouds
			if (_render_mesh == Render::NONE) {
				_point_renderer->insertPoints("frame_registered_A", deformed_points, ml::RGBColor::Cyan, 0.001f);
				_point_renderer->insertPoints("frame_registered_B", registration->getTarget(), ml::RGBColor::Green, 0.001f, false);
			}
			else {
				_point_renderer->removePoints("frame_registered_A");
				_point_renderer->removePoints("frame_registered_B");
			}
			// render mesh
			if (_render_mesh == Render::DEFORMATION) {
				_mesh_renderer->insertMesh("mesh_a", deformed_points, ml::RGBColor::Cyan.toVec4f());
				_mesh_renderer->removeMesh("mesh_b");
				_point_renderer->insertPoints("frame_registered_B", registration->getTarget(), ml::RGBColor::Green, 0.001f, false);
			}
			else if (_render_mesh == Render::ALL) {
				_mesh_renderer->insertMesh("mesh_a", deformed_points, ml::RGBColor::Cyan.toVec4f());
				_mesh_renderer->insertMesh("mesh_b", registration->getTarget(), ml::RGBColor::Green.toVec4f(), false);
			}
			else if (_render_mesh == Render::TARGET) {
				_mesh_renderer->insertMesh("mesh_b", registration->getTarget(), ml::RGBColor::Green.toVec4f(), false);
				_mesh_renderer->removeMesh("mesh_a");
			}
		}

		// fixed positions
		std::vector<Point> render_fixed_positions = registration->getFixedPostions();
		if (!render_fixed_positions.empty())
			_point_renderer->insertPoints("frame_fixed_positions", render_fixed_positions, ml::RGBColor::Red, 0.005f);

		// deformation graph
		if (_render_deformation_graph) {
			auto render_dg = registration->getDeformationGraphMesh();
			_point_renderer->insertMesh("deformation_graph", render_dg, 0.001f, false);
		}
		else {
			_point_renderer->removePoints("deformation_graph");
		}
	}
}


void RenderRegistration::renderRegistrationSequence(std::shared_ptr<SequenceRegistration> sequence_registration)
{
	if (sequence_registration) {
		auto current = sequence_registration->getCurrent();

		if (_reigistration_finished) {
			_mesh_renderer->removeMesh("target");
			_mesh_renderer->removeMesh("deformed");

			for (int i = 0; i < current; ++i) {
				std::string key = "mesh_" + std::to_string(i);
				if (!_mesh_renderer->keyExists(key)) {
					auto deformed_points = sequence_registration->getDeformedMesh(i);
					_mesh_renderer->insertMesh(key, deformed_points, ml::RGBColor::Cyan.toVec4f(), false);
				}
			}
		}
		else {
			_reigistration_finished = sequence_registration->finished();
			_mesh_renderer->insertMesh("target", sequence_registration->getMesh(current), ml::RGBColor::Green.toVec4f());
			//_point_renderer->insertPoints("target", _register_sequence_of_frames->getMesh(_register_sequence_of_frames->getCurrent()), ml::RGBColor::Yellow);

			auto deformed_points = sequence_registration->getDeformedMesh(current);
			_mesh_renderer->insertMesh("deformed", deformed_points, ml::RGBColor::Cyan.toVec4f());
		}

		// deformation graph
		if (_render_deformation_graph && !_reigistration_finished) {
			auto render_dg = sequence_registration->getDeformationGraphMesh(sequence_registration->getCurrent());
			_point_renderer->insertMesh("deformation_graph", render_dg, 0.001f, false, true);
		}
		else {
			_point_renderer->removePoints("deformation_graph");
		}
	}
}


void RenderRegistration::renderError(std::vector<std::pair<Point, Point>> error_points)
{
	if (_render_error) {
		auto distance_errors = evaluate_distance_error(error_points);
		float average = std::accumulate(distance_errors.begin(), distance_errors.end(), 0.0) / static_cast<float>(distance_errors.size());
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
		_point_renderer->insertLine("error", edges, 0.0005);
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
	_current_reference_frame = 0;
	_reigistration_finished = false;
}