#include "icp.h"
#include "stdafx.h"
#include "algo/icp-ceres.h"
#include "algo/eigen_quaternion.h"

Vector3d vec3f_to_vector3d(const ml::vec3f &vec)
{
	return Vector3d(vec.x, vec.y, vec.z);
}

ml::vec3f vector3d_to_vec3f(const Vector3d &vec)
{
	return ml::vec3f(vec[0], vec[1], vec[2]);
}

std::vector<Vector3d> vector_vec3f_to_vector_vector3d(std::vector<ml::vec3f> & vec)
{
	std::vector<Vector3d> converted_vec;
	std::transform(vec.begin(), vec.end(),
				   std::back_inserter(converted_vec), &vec3f_to_vector3d);
	return converted_vec;
}

std::vector<ml::vec3f> vector_vector3d_to_vector_vec3f(std::vector<Vector3d> & vec)
{
	std::vector<ml::vec3f> converted_vec;
	std::transform(vec.begin(), vec.end(),
				   std::back_inserter(converted_vec), &vector3d_to_vec3f);
	return converted_vec;
}

Vector4d to_vector4d(Vector3d vec)
{
	return Vector4d(vec[0], vec[1], vec[2], 1);
}

ml::mat4f matrix4d_to_mat4f(Eigen::Matrix4d mat)
{
	return ml::mat4f(mat(0, 0), mat(0, 1), mat(0, 2), mat(0, 3),
					 mat(1, 0), mat(1, 1), mat(1, 2), mat(1, 3),
					 mat(2, 0), mat(2, 1), mat(2, 2), mat(2, 3),
					 mat(3, 0), mat(3, 1), mat(3, 2), mat(3, 3));
}

ml::mat4f iterative_closest_point(std::vector<ml::vec3f> & points_a, std::vector<ml::vec3f> & points_b)
{
	auto points_frame_A = vector_vec3f_to_vector_vector3d(points_a);
	auto points_frame_B = vector_vec3f_to_vector_vector3d(points_b);
	auto icp = ICP_Ceres::pointToPoint_SophusSE3(points_frame_A, points_frame_B);

	Eigen::Matrix4d transform = icp.matrix();
	return matrix4d_to_mat4f(transform);

}