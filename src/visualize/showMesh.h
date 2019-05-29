#pragma once
#include "../mLibInclude.h"
#include "i_showData.h"
#include "input_reader/i_reader.h"
#include "meshRenderer.h"
#include "pointsRenderer.h"
#include "input_reader/mesh_reader.h"

#include "algo/file_writer.h"
#include "algo/evaluate_registration.h"
#include "algo/registration/registration.h"
#include "algo/registration/non_rigid_registration_all_frames.h"

#include "algo/registration/arap/arap.h"

enum Render
{
	NONE,
	DEFORMATION,
	TARGET,
	ALL
};

class ShowMesh : public IShowData
{
private:
	std::shared_ptr<FileWriter> _logger;
	std::unique_ptr<PointsRenderer> _point_renderer;
	std::unique_ptr<MeshRenderer> _mesh_renderer;
	std::unique_ptr<IMeshReader> _input_mesh;
	std::unique_ptr<IMeshReader> _reference_registration_mesh;
	unsigned int _number_of_nodes;
	unsigned int _current_frame;
	std::vector<unsigned int> _selected_frame_for_registration;
	bool _solve_registration;
	RegistrationType _registration_type;
	bool _render_points = true;
	Render _render_mesh = Render::ALL;
	bool _render_reference_mesh;
	bool _render_error;	
	bool _render_deformation_graph;
	bool _calculate_error;
	//std::unique_ptr<IRegistration> _registration;
	std::unique_ptr<IRegistration> _registration;
	//std::unique_ptr<NonRigidRegistrationAllFrames<AsRigidAsPossible, DeformationGraph<ARAPGraph, ARAPNode>>> _registration_frames;
	//std::unique_ptr < NonRigidRegistrationAllFrames < ED::EmbeddedDeformation, DeformationGraph<ED::Graph, ED::Node>>> _registration_frames;
	std::unique_ptr<ErrorEvaluation> _error_evaluation;
private:
	void renderRegistrationTwoFrames();
	void renderRegistrationAllFrames();
	void renderCurrentMesh();
	void renderError();
private:
	void renderRegistration();
	void nonRigidRegistration();
	void solveAllNonRigidRegistration();
public:
	void init(ml::ApplicationData &app) override;
	void render(ml::Cameraf& camera) override;
	void key(UINT key) override;
};
