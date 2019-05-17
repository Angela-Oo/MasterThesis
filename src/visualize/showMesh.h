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
#include "algo/registration/as_rigid_as_possible.h"
#include "algo/registration/embedded_deformation.h"
#include "algo/registration/non_rigid_registration_all_frames.h"


class ShowMesh : public IShowData
{
private:
	std::shared_ptr<FileWriter> _logger;
	std::unique_ptr<PointsRenderer> _point_renderer;
	std::unique_ptr<MeshRenderer> _mesh_renderer;
	std::unique_ptr<IMeshReader> _input_mesh;
	std::unique_ptr<IMeshReader> _reference_registration_mesh;
	const int _number_of_deformation_graph_nodes = 2000;
	unsigned int _current_frame = 0;
	std::vector<unsigned int> _selected_frame_for_registration;
	bool _solve_registration = false;
	RegistrationType _registration_type;
	bool _render_points = true;
	bool _render_mesh = true;
	bool _render_reference_mesh = true;
	bool _render_error = false;	
	bool _render_deformation_graph = true;
	bool _calculate_error = false;
	std::unique_ptr<IRegistration> _registration;
	std::unique_ptr<NonRigidRegistrationAllFrames<AsRigidAsPossible, DeformationGraph<ARAPGraph, ARAPNode>>> _registration_frames;
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
