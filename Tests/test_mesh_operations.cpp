#include "pch.h"

#include "mLibInclude.h"
#include "mesh/mesh_definition.h"
#include "algo/surface_mesh/mesh_operations.h"



// Mesh structure
//         v0
//       /    \
//     v2 ---- v1
//
class MeshOperationsFace : public ::testing::Test {
public:
	SurfaceMesh mesh;
	std::vector<vertex_descriptor> vids;
	std::vector<face_descriptor> fids;
public:
	MeshOperationsFace() {
		// initialization code here

		vids.push_back(mesh.add_vertex(Point(0., 3., 0.)));
		vids.push_back(mesh.add_vertex(Point(-2., 0., 0.)));
		vids.push_back(mesh.add_vertex(Point(2., 0., 0.)));

		fids.push_back(mesh.add_face(vids[0], vids[1], vids[2]));
	}

	void SetUp() {
		// code here will execute just before the test ensues 
	}

	void TearDown() {
		// code here will be called just after the test completes
		// ok to through exceptions from here if need be
	}

	~MeshOperationsFace() {
		// cleanup any pending stuff, but no exceptions allowed
	}

	// put in any custom data members that you need 
};


TEST_F(MeshOperationsFace, TestMesh)
{
	EXPECT_EQ(mesh.number_of_faces(), 1);
	EXPECT_EQ(mesh.number_of_vertices(), 3);
}


TEST_F(MeshOperationsFace, TestSplitEdgeAtCenter)
{
	auto he = mesh.halfedge(fids[0]);
	auto e = mesh.edge(he);

	Registration::splitEdgeAtCenter(e, mesh);

	EXPECT_EQ(mesh.number_of_faces(), 1);
	EXPECT_EQ(mesh.number_of_vertices(), 4);

	face_descriptor f = *mesh.faces().begin();
	std::vector<vertex_descriptor> vertices;
	for (auto & v : mesh.vertices())
		vertices.push_back(v);

	auto new_v = mesh.point(vertices[3]);
	EXPECT_EQ(new_v, Point(1., 1.5, 0.));

	std::vector<vertex_descriptor> vfs;
	for (auto & v : mesh.vertices_around_face(mesh.halfedge(f)))
		vfs.push_back(v);

	EXPECT_EQ(vfs.size(), 4);
	EXPECT_EQ(vfs[0], vertices[0]);
	EXPECT_EQ(vfs[1], vertices[1]);
	EXPECT_EQ(vfs[2], vertices[2]);
	EXPECT_EQ(vfs[3], vertices[3]);
}

TEST_F(MeshOperationsFace, TestSplitFace2Vertices)
{
	auto f = *mesh.faces().begin();

	auto e0 = mesh.edge(mesh.halfedge(vids[0]));
	auto e1 = mesh.edge(mesh.halfedge(vids[1]));

	halfedge_descriptor he0 = Registration::splitEdgeAtCenter(e0, mesh);
	EXPECT_EQ(mesh.point(mesh.target(he0)), Point(-1., 1.5, 0.));

	halfedge_descriptor he1 = Registration::splitEdgeAtCenter(e1, mesh);
	EXPECT_EQ(mesh.point(mesh.target(he1)), Point(0., 0., 0.));

	Registration::splitFaceAtEdge(f, mesh.target(he0), mesh.target(he1), mesh);

	EXPECT_EQ(mesh.number_of_faces(), 2);
	EXPECT_EQ(mesh.number_of_vertices(), 5);
	EXPECT_EQ(mesh.number_of_edges(), 6);

	std::vector<face_descriptor> faces;
	for (auto & f : mesh.faces())
		faces.push_back(f);

	std::vector<vertex_descriptor> vertices;
	for (auto & v : mesh.vertices())
		vertices.push_back(v);

	auto new_he = mesh.halfedge(vertices[3], vertices[4]);
	EXPECT_TRUE(mesh.is_valid(new_he));
	EXPECT_EQ(mesh.point(vertices[3]), Point(-1., 1.5, 0.));
	EXPECT_EQ(mesh.point(vertices[4]), Point(0., 0., 0.));
}


TEST_F(MeshOperationsFace, TestSplitFace)
{
	auto f = *mesh.faces().begin();

	Registration::splitFaceAtEdge(f, mesh);

	EXPECT_EQ(mesh.number_of_faces(), 4);
	EXPECT_EQ(mesh.number_of_vertices(), 6);

	auto v_it = mesh.vertices().begin() + 3;
	EXPECT_EQ(mesh.point(*v_it), Point(1., 1.5, 0.));
	++v_it;
	EXPECT_EQ(mesh.point(*v_it), Point(-1., 1.5, 0.));
	++v_it;
	EXPECT_EQ(mesh.point(*v_it), Point(0., 0., 0.));
}



// Mesh structure
//          v0
//       /   |   \
//     v3    |    v1
//       \   |   /
//          v2
class MeshOperationsOneEdge : public ::testing::Test {
public:
	SurfaceMesh mesh;
	std::vector<vertex_descriptor> vids;
	std::vector<face_descriptor> fids;
public:
	MeshOperationsOneEdge() {
		// initialization code here

		vids.push_back(mesh.add_vertex(Point(0., 2., 0.)));
		vids.push_back(mesh.add_vertex(Point(3., 0., 0.)));
		vids.push_back(mesh.add_vertex(Point(0., -2., 0.)));
		vids.push_back(mesh.add_vertex(Point(-3., 0., 0.)));

		fids.push_back(mesh.add_face(vids[0], vids[1], vids[2]));
		fids.push_back(mesh.add_face(vids[0], vids[2], vids[3]));
	}

	void SetUp() {
		// code here will execute just before the test ensues 
	}

	void TearDown() {
		// code here will be called just after the test completes
		// ok to through exceptions from here if need be
	}

	~MeshOperationsOneEdge() {
		// cleanup any pending stuff, but no exceptions allowed
	}

	// put in any custom data members that you need 
};


TEST_F(MeshOperationsOneEdge, TestMesh)
{
	EXPECT_EQ(mesh.number_of_faces(), 2);
	EXPECT_EQ(mesh.number_of_vertices(), 4);

	auto he = mesh.halfedge(fids[0]);
	auto s = mesh.source(he);
	auto t = mesh.target(he);
	EXPECT_EQ(vids[2], s);
	EXPECT_EQ(vids[0], t);
}

TEST_F(MeshOperationsOneEdge, TestSplitEdge)
{
	auto he = mesh.halfedge(fids[0]);
	auto e = mesh.edge(he);

	Registration::splitEdge(e, mesh);

	EXPECT_EQ(mesh.number_of_faces(), 4);
	EXPECT_EQ(mesh.number_of_vertices(), 5);

	std::vector<face_descriptor> faces;
	for (auto & f : mesh.faces())
		faces.push_back(f);

	std::vector<vertex_descriptor> vertices;
	for (auto & v : mesh.vertices())
		vertices.push_back(v);

	auto last_vid = vertices[vertices.size() -1];

	auto new_v = mesh.point(last_vid);
	EXPECT_EQ(new_v, Point(0., 0., 0.));

	std::vector<vertex_descriptor> vfs;
	for (auto & v : mesh.vertices_around_face(mesh.halfedge(faces[0])))
		vfs.push_back(v);

	EXPECT_EQ(vfs.size(), 3);
	EXPECT_EQ(vfs[0], vertices[1]);
	EXPECT_EQ(vfs[1], vertices[2]);
	EXPECT_EQ(vfs[2], vertices[4]);

	vfs.clear();
	for (auto & v : mesh.vertices_around_face(mesh.halfedge(faces[1])))
		vfs.push_back(v);
	EXPECT_EQ(vfs.size(), 3);
	EXPECT_EQ(vfs[0], vertices[3]);
	EXPECT_EQ(vfs[1], vertices[0]);
	EXPECT_EQ(vfs[2], vertices[4]);

	vfs.clear();
	for (auto & v : mesh.vertices_around_face(mesh.halfedge(faces[2])))
		vfs.push_back(v);
	EXPECT_EQ(vfs.size(), 3);
	EXPECT_EQ(vfs[0], vertices[4]);
	EXPECT_EQ(vfs[1], vertices[0]);
	EXPECT_EQ(vfs[2], vertices[1]);

	vfs.clear();
	for (auto & v : mesh.vertices_around_face(mesh.halfedge(faces[2])))
		vfs.push_back(v);
	EXPECT_EQ(vfs.size(), 3);
	EXPECT_EQ(vfs[0], vertices[4]);
	EXPECT_EQ(vfs[1], vertices[0]);
	EXPECT_EQ(vfs[2], vertices[1]);
}



TEST_F(MeshOperationsOneEdge, TestSplitEdgeAtCenter)
{
	auto he = mesh.halfedge(fids[0]);
	auto e = mesh.edge(he);

	Registration::splitEdgeAtCenter(e, mesh);

	EXPECT_EQ(mesh.number_of_faces(), 2);
	EXPECT_EQ(mesh.number_of_vertices(), 5);

	std::vector<face_descriptor> faces;
	for (auto & f : mesh.faces())
		faces.push_back(f);

	std::vector<vertex_descriptor> vertices;
	for (auto & v : mesh.vertices())
		vertices.push_back(v);
	
	auto new_v = mesh.point(vertices[4]);
	EXPECT_EQ(new_v, Point(0., 0., 0.));

	std::vector<vertex_descriptor> vfs;
	for (auto & v : mesh.vertices_around_face(mesh.halfedge(faces[0])))
		vfs.push_back(v);

	EXPECT_EQ(vfs.size(), 4);
	EXPECT_EQ(vfs[0], vertices[0]);
	EXPECT_EQ(vfs[1], vertices[1]);
	EXPECT_EQ(vfs[2], vertices[2]);
	EXPECT_EQ(vfs[3], vertices[4]);

	vfs.clear();
	for (auto & v : mesh.vertices_around_face(mesh.halfedge(faces[1])))
		vfs.push_back(v);
	EXPECT_EQ(vfs.size(), 4);
	EXPECT_EQ(vfs[0], vertices[0]);
	EXPECT_EQ(vfs[1], vertices[4]);
	EXPECT_EQ(vfs[2], vertices[2]);
	EXPECT_EQ(vfs[3], vertices[3]);	
}





// Mesh structure
//         v0
//       /    \
//     v1 ---- v2
//   /   \    /  \
// v3 ---- v4 ---- v5
class MeshOperationsThreeEdges : public ::testing::Test {
public:
	SurfaceMesh mesh;
	std::vector<vertex_descriptor> vids;
	std::vector<face_descriptor> fids;
public:
	MeshOperationsThreeEdges() {
		// initialization code here

		vids.push_back(mesh.add_vertex(Point(0., 4., 0.)));
		vids.push_back(mesh.add_vertex(Point(-2., 1., 0.)));
		vids.push_back(mesh.add_vertex(Point(2., 1., 0.)));
		vids.push_back(mesh.add_vertex(Point(-4., -2., 0.)));
		vids.push_back(mesh.add_vertex(Point(4., -2., 0.)));
		vids.push_back(mesh.add_vertex(Point(0., -2., 0.)));

		fids.push_back(mesh.add_face(vids[0], vids[2], vids[1]));
		fids.push_back(mesh.add_face(vids[1], vids[2], vids[4]));
		fids.push_back(mesh.add_face(vids[1], vids[4], vids[3]));
		fids.push_back(mesh.add_face(vids[2], vids[5], vids[4]));
	}

	void SetUp() {
		// code here will execute just before the test ensues 
	}

	void TearDown() {
		// code here will be called just after the test completes
		// ok to through exceptions from here if need be
	}

	~MeshOperationsThreeEdges() {
		// cleanup any pending stuff, but no exceptions allowed
	}

	// put in any custom data members that you need 
};



TEST_F(MeshOperationsThreeEdges, TestMesh)
{
	EXPECT_EQ(mesh.number_of_faces(), 4);
	EXPECT_EQ(mesh.number_of_vertices(), 6);
}

TEST_F(MeshOperationsThreeEdges, TestSplitTriangle)
{
	auto he = mesh.halfedge(fids[0]);
	auto f = mesh.face(he);

	Registration::splitFaceAtEdge(f, mesh);

	EXPECT_EQ(mesh.number_of_faces(), 7);
	EXPECT_EQ(mesh.number_of_vertices(), 9);
}


