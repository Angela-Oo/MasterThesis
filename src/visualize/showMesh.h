#pragma once
#include "../mLibInclude.h"
#include "i_showData.h"
#include "input_reader/i_reader.h"
#include "meshRenderer.h"
#include "pointsRenderer.h"
#include "input_reader/mesh_reader.h"
#include "algo/registration.h"


class ShowMesh : public IShowData
{
private:
	std::unique_ptr<PointsRenderer> _point_renderer;
	std::unique_ptr<MeshRenderer> _mesh_renderer;
	std::unique_ptr<MeshReader> _input_mesh;
	std::unique_ptr<MeshReader> _reference_registration_mesh;
	unsigned int _current_frame = 0;
	std::vector<unsigned int> _selected_frame_for_registration;
	bool _solve_non_rigid_registration = false;
	bool _solve_all_non_rigid_registration = false;
	bool _render_mesh = true;
	bool _render_reference_mesh = true;
	std::unique_ptr<IRegistration> _registration;
	std::unique_ptr<NonRigidRegistrationFrames> _registration_frames;
private:
	void renderMesh();
	void renderRegisteredPoints();
	void renderError();
	void renderRegistration();
	void nonRigidRegistration(int frame_a, int frame_b);
	void solveAllNonRigidRegistration();
public:
	void init(ml::ApplicationData &app) override;
	void render(ml::Cameraf& camera) override;
	void key(UINT key) override;

};
