#include "pch.h"

#include "mLibInclude.h"
//#include "mLibCore.h"
//#include "mLibCGAL.h"
#include "algo/surface_mesh/mesh_definition.h"
#include "algo/registration/deformation_graph/refine_deformation_graph.h"

TEST(TestCaseName, TestName) {
  EXPECT_EQ(1, 1);
  EXPECT_TRUE(true);
}


//          v0
//       /   |   \
//     v3    |    v1
//       \   |   /
//          v2
TEST(Refine, TestOneEdge) {
	SurfaceMesh mesh;

	
	std::vector<vertex_descriptor> vids;
	vids.push_back(mesh.add_vertex(Point( 0.,  2., 0.)));
	vids.push_back(mesh.add_vertex(Point( 3.,  0., 0.)));
	vids.push_back(mesh.add_vertex(Point( 0., -2., 0.)));
	vids.push_back(mesh.add_vertex(Point(-3.,  0., 0.)));

	mesh.add_face(vids[0], vids[1], vids[2]);
	mesh.add_face(vids[0], vids[2], vids[3]);

	EXPECT_EQ(mesh.number_of_faces(), 2);
	EXPECT_EQ(mesh.number_of_vertices(), 4);

	auto smooth_cost_property_map = mesh.add_property_map<edge_descriptor, double>("e:smooth_cost", 0.);
	auto e = mesh.edge(mesh.halfedge(vids[2]));
	smooth_cost_property_map.first[e] = 1.;

	//auto edges = Registration::getEdgesToRefine(mesh);
	//EXPECT_EQ(edges.size(), 1);
	//EXPECT_EQ(edges[0], e);
}


TEST(Refine, TestThreeEdges) {
	SurfaceMesh mesh;

	std::vector<vertex_descriptor> vids;
	vids.push_back(mesh.add_vertex(Point( 0.,  4., 0.)));
	vids.push_back(mesh.add_vertex(Point( 2.,  1., 0.)));
	vids.push_back(mesh.add_vertex(Point(-2.,  1., 0.)));
	vids.push_back(mesh.add_vertex(Point( 4., -2., 0.)));
	vids.push_back(mesh.add_vertex(Point( 0., -2., 0.)));
	vids.push_back(mesh.add_vertex(Point(-4., -2., 0.)));

	mesh.add_face(vids[0], vids[1], vids[2]);
	mesh.add_face(vids[1], vids[3], vids[4]);
	mesh.add_face(vids[1], vids[4], vids[2]);
	mesh.add_face(vids[2], vids[4], vids[5]);

	EXPECT_EQ(mesh.number_of_faces(), 4);
	EXPECT_EQ(mesh.number_of_vertices(), 6);
}