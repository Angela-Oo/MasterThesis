#include "stdafx.h"
#include "se3.h"

#include <ceres/rotation.h>

ml::mat4f rigid_transformation_from_se3(ml::vec6d & rotation_translation)
{
	double rotation_matrix[9];
	ceres::AngleAxisToRotationMatrix(rotation_translation.array, rotation_matrix);

	// rotation
	ml::mat4f rotation = ml::mat4f::identity();
	rotation(0, 0) = rotation_matrix[0];
	rotation(1, 0) = rotation_matrix[1];
	rotation(2, 0) = rotation_matrix[2];
	rotation(0, 1) = rotation_matrix[3];
	rotation(1, 1) = rotation_matrix[4];
	rotation(2, 1) = rotation_matrix[5];
	rotation(0, 2) = rotation_matrix[6];
	rotation(1, 2) = rotation_matrix[7];
	rotation(2, 2) = rotation_matrix[8];

	// translation
	ml::vec3d translation_vector = { rotation_translation[3], rotation_translation[4], rotation_translation[5] };
	ml::mat4f translation = ml::mat4f::translation(translation_vector);

	return translation * rotation;
}
