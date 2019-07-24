#pragma once

#include "algo/file_writer.h"
#include "algo/registration/i_registration.h"
#include <ceres/ceres.h>

namespace Registration {

void logRegistrationOptions(std::shared_ptr<FileWriter> logger, const RegistrationOptions & registration_options);
void logCeresOptions(std::shared_ptr<FileWriter> logger, const ceres::Solver::Options & ceres_options);

}