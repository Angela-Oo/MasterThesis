#pragma once

#include "mLibCore.h"
#include "mesh/mesh_definition.h"

Matrix convertMatrix(const ml::mat3d &m);

ml::mat3d convertMatrix(const Matrix &m);

Vector convertVector(const ml::vec3d & v);

ml::vec3d convertVector(const Vector & v);