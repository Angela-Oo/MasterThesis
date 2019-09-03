#pragma once
#include "algo/registration/interface/registration_options.h"

Registration::Input inputById(std::string input_id);

Registration::RegistrationOptions parse(int argc, char* argv[]);


