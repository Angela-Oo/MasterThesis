#pragma once
#include "../mLibInclude.h"
#include "i_showData.h"
#include "input_reader/i_reader.h"
#include "render/render_registration.h"
#include "input_reader/mesh_reader.h"

#include "algo/file_writer.h"
#include "algo/evaluate_registration.h"
#include "algo/registration/registration.h"
#include "algo/registration/sequence_registration/sequence_registration.h"

#include "algo/registration/arap/arap.h"


class ShowMesh : public IShowData
{
private:
	std::shared_ptr<FileWriter> _logger;
	std::unique_ptr<RenderRegistration> _renderer;
	std::shared_ptr<IMeshReader> _input_mesh;
	std::shared_ptr<IMeshReader> _reference_registration_mesh;
	std::vector<unsigned int> _selected_frame_for_registration;
	bool _solve_registration;
	RegistrationType _registration_type;
	std::string _save_images_folder;
	std::string _image_name;
	std::string _data_name;
	bool _calculate_error;
	std::shared_ptr<IRegistration> _registration;
	std::shared_ptr<SequenceRegistration> _register_sequence_of_frames;
	std::unique_ptr<ErrorEvaluation> _error_evaluation;	
	RegistrationOptions _registration_options;
private:
	void renderCurrentMesh();
	void renderError();
	std::string getImageFolderName(RegistrationType type);
private:
	void renderRegistration();
	void nonRigidRegistration();	
	void solveAllNonRigidRegistration();
public:
	void init(ml::ApplicationData &app) override;
	void render(ml::Cameraf& camera) override;
	void registration();
	void key(UINT key) override;
};
