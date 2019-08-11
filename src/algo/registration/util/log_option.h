#pragma once

#include "util/file_writer.h"
#include "algo/registration/interface/i_registration.h"
#include <ceres/ceres.h>

namespace Registration {

void logRegistrationOptions(std::shared_ptr<FileWriter> logger, const RegistrationOptions & registration_options);
void logCeresOptions(std::shared_ptr<FileWriter> logger, const ceres::Solver::Options & ceres_options);

void logOptions(std::shared_ptr<FileWriter> logger, const RegistrationOptions & registration_options, const ceres::Solver::Options & ceres_options);

}
