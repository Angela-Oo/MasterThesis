#pragma once
#include "../mLibInclude.h"
#include "i_showData.h"
#include "input_reader/i_reader.h"
#include "render/render_registration.h"
#include "input_reader/mesh_reader.h"

#include "util/file_writer.h"
#include "algo/registration_evaluation/evaluate_registration.h"
#include "algo/registration/interface/i_registration.h"
#include "algo/registration/interface/registration_type.h"
#include "algo/registration/sequence_registration/sequence_registration.h"


struct Arguments
{
	RegistrationOptions registration_options;

};

class AppRegistration : public IShowData
{
private:
	std::shared_ptr<FileWriter> _logger;
	std::unique_ptr<RenderRegistration> _renderer;
	std::shared_ptr<IMeshReader> _mesh_reader;
	std::string _save_images_folder;
	std::string _image_name;
	std::shared_ptr<IRegistration> _registration;
	std::shared_ptr<ISequenceRegistration> _register_sequence_of_frames;
	std::unique_ptr<ErrorEvaluation> _error_evaluation;
	RegistrationOptions _options;
	bool _finished{ false };
private:
	void renderCurrentMesh();
	void renderError();
private:
	void renderRegistration();
	void nonRigidRegistration();
	void solveAllNonRigidRegistration();
	void initMeshReader();
	void initRegistration();
	void registration();
public:	
	void render(ml::Cameraf& camera) override;	
	void key(UINT key) override;
	void init(ml::ApplicationData &app) override;
public:
	AppRegistration(const RegistrationOptions & options);
};
