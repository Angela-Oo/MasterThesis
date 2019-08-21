#pragma once

#include <memory>

namespace Registration {

template<typename Deformation>
Deformation adaptRigidity(const Deformation & deformation) {
	return deformation;
}

}

