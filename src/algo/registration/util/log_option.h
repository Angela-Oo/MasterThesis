#pragma once


#include "algo/registration/interface/i_registration.h"
#include "algo/registration/interface/registration_options.h"
#include "util/file_writer.h"
#include "util/ceres_include.h"

namespace Registration {

void logRegistrationOptions(std::shared_ptr<FileWriter> logger, const RegistrationOptions & registration_options);
void logCeresOptions(std::shared_ptr<FileWriter> logger, const ceres::Solver::Options & ceres_options);

void logOptions(std::shared_ptr<FileWriter> logger, const RegistrationOptions & registration_options, const ceres::Solver::Options & ceres_options);

}
