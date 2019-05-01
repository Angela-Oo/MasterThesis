#include "evaluate_registration.h"



class Plane
{
public:
	OpenMesh::Vec3f point;
	OpenMesh::Vec3f normal;
public:
	Plane(OpenMesh::Vec3f p, OpenMesh::Vec3f n)
		: point(p)
		, normal(n)
	{}
};

OpenMesh::Vec3f pointOnPlane(Plane plane, OpenMesh::Vec3f point)
{
	auto normal = plane.normal.normalized();
	auto direction = point - plane.point;
	auto angle = OpenMesh::dot(normal, direction.normalized());
	auto h = sin(angle) * direction.length();

	if (isnan(h))
		std::cout <<"fuu h" << std::endl;
	auto point_on_plane = point - normal * h;
	return point_on_plane;
}

float area(OpenMesh::Vec3f point_a, OpenMesh::Vec3f point_b, OpenMesh::Vec3f point_c)
{
	auto c = point_b - point_a;
	auto b = point_c - point_a;
	assert(c.length() != 0.f && b.length() != 0.f);

	auto angle = OpenMesh::dot(c.normalized(), b.normalized());
	auto hb = sin(angle) * b.length();
	auto area = (hb * b.length()) / 2.;
	return area;
}

OpenMesh::Vec3f barycentricCoordinates(OpenMesh::Vec3f point_a, OpenMesh::Vec3f point_b, OpenMesh::Vec3f point_c, OpenMesh::Vec3f point_on_triangle)
{
	auto area_u = area(point_on_triangle, point_a, point_b);
	auto area_v = area(point_on_triangle, point_b, point_c);
	auto area_w = area(point_on_triangle, point_c, point_a);
	auto area_face = area(point_a, point_b, point_c);

	assert(area_face != 0.);
	float u = area_u / area_face;
	float v = area_v / area_face;
	float w = area_w / area_face;
	return OpenMesh::Vec3f(u, v, w);
}

OpenMesh::Vec3f barycentricCoordinates(ml::OpenMeshTriMesh::Mesh & mesh, OpenMesh::FaceHandle face_handle, OpenMesh::Vec3f point_on_plane)
{
	auto vertex_face_iter = mesh.cfv_iter(face_handle);
	auto point_a = mesh.point(vertex_face_iter.handle());
	auto point_b = mesh.point((++vertex_face_iter).handle());
	auto point_c = mesh.point((++vertex_face_iter).handle());

	return barycentricCoordinates(point_a, point_b, point_c, point_on_plane);
}


OpenMesh::Vec3f getNearestPointOnSurface(ml::OpenMeshTriMesh::Mesh & mesh, OpenMeshKNN& knn, OpenMesh::Vec3f point)
{
	mesh.request_face_normals();
	
	auto v_handle = knn.nearest_index(point);
	auto vf_begin = mesh.vf_iter(v_handle);
	auto v_point = mesh.point(v_handle);

	for (auto vfIt = vf_begin; vfIt; ++vfIt)
	{
		auto normal = mesh.calc_face_normal(vfIt.handle());
		normal = normal.normalized();
		auto point_on_plane = pointOnPlane(Plane(v_point, normal), point);
		auto barycentric_coordinates = barycentricCoordinates(mesh, vfIt.handle(), point_on_plane);
		float relative_area = barycentric_coordinates.l1_norm();
		if (relative_area <= 1) {
			return point_on_plane;
		}
	}
	return v_point;
}

std::vector<ml::vec3f> evaluate_error(const Mesh & result, const Mesh & reference_mesh)
{
	ml::OpenMeshTriMesh::Mesh mesh;
	ml::OpenMeshTriMesh::convertToOpenMesh(reference_mesh, mesh);
	OpenMeshKNN knn(mesh);  	

	std::vector<ml::vec3f> nearest_points;
	for (auto & p : result.m_vertices) {
		auto nearest_point = getNearestPointOnSurface(mesh, knn, OpenMesh::Vec3f(p.position[0], p.position[1], p.position[2]));
		if (nearest_point.length() == 0.f || isnan(nearest_point[0]) || isnan(nearest_point[1]))
			std::cout << "help" << std::endl;
		nearest_points.push_back(ml::vec3f(nearest_point[0], nearest_point[1], nearest_point[2]));
	}
	return nearest_points;
}


std::vector<float> evaluate_distance_error(const Mesh & points_a, const std::vector<ml::vec3f>& points_b)
{
	assert(points_a.m_vertices.size() == points_b.size());
	std::vector<float> distances;
	for (int i = 0; i < points_a.m_vertices.size(); ++i) {
		auto vector = points_b[i] - points_a.m_vertices[i].position;
		auto distance = (vector).lengthSq();
		if (isnan(distance) || isinf(distance))
			std::cout << "bad" << std::endl;
		distances.push_back(distance);
	}
	return distances;
}







std::vector<ml::vec3f> ErrorEvaluation::evaluate_error(const Mesh & mesh)
{
	std::vector<ml::vec3f> nearest_points;
	for (auto & p : mesh.m_vertices) {
		auto nearest_point = getNearestPointOnSurface(_reference_open_mesh, *_knn.get(), OpenMesh::Vec3f(p.position[0], p.position[1], p.position[2]));
		if (nearest_point.length() == 0.f || isnan(nearest_point[0]) || isnan(nearest_point[1]))
			std::cout << "help" << std::endl;
		nearest_points.push_back(ml::vec3f(nearest_point[0], nearest_point[1], nearest_point[2]));
	}
	return nearest_points;
}

ErrorEvaluation::ErrorEvaluation(const Mesh & reference_mesh)
	: _reference_mesh(reference_mesh)
{
	ml::OpenMeshTriMesh::convertToOpenMesh(reference_mesh, _reference_open_mesh);
	_knn = std::make_unique<OpenMeshKNN>(_reference_open_mesh);
}