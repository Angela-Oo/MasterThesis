#include "pch.h"

#include "mLibInclude.h"
#include "mesh/mesh_definition.h"
#include "algo/registration/deformation_graph/refine_deformation_graph.h"



// Mesh structure
//          v0
//       /   |   \
//     v3    |    v1
//       \   |   /
//          v2
class RefineOneEdge : public ::testing::Test {
public:
	SurfaceMesh mesh;
	std::vector<vertex_descriptor> vids;
	std::vector<face_descriptor> fids;
public:
	RefineOneEdge() {
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

	~RefineOneEdge() {
		// cleanup any pending stuff, but no exceptions allowed
	}

// put in any custom data members that you need 
};


TEST_F(RefineOneEdge, TestEdgesToRefine)
{
	auto smooth_cost_property_map = mesh.add_property_map<edge_descriptor, double>("e:smooth_cost", 0.);
	auto he = mesh.halfedge(fids[0]);
	auto e = mesh.edge(he);
	smooth_cost_property_map.first[e] = 1.;

	auto edges = Registration::getEdgesToRefine(mesh);
	EXPECT_EQ(edges.size(), 1);
	EXPECT_EQ(edges[0], e);
}

//TEST_F(RefineOneEdge, TestRefine)
//{
//	auto smooth_cost_property_map = mesh.add_property_map<edge_descriptor, double>("e:smooth_cost", 0.);
//	auto he = mesh.halfedge(fids[0]);
//
//	auto e = mesh.edge(he);
//	smooth_cost_property_map.first[e] = 1.;
//
//	auto refined = Registration::refineDeformationGraph(mesh);
//
//	EXPECT_EQ(refined.number_of_faces(), 4);
//	EXPECT_EQ(refined.number_of_vertices(), 5);
//}








// Mesh structure
//         v0
//       /    \
//     v1 ---- v2
//   /   \    /  \
// v3 ---- v4 ---- v5
class RefineThreeEdges : public ::testing::Test {
public:
	SurfaceMesh mesh;
	std::vector<vertex_descriptor> vids;
	std::vector<face_descriptor> fids;
public:
	RefineThreeEdges() {
		// initialization code here

		vids.push_back(mesh.add_vertex(Point( 0.,  4., 0.)));
		vids.push_back(mesh.add_vertex(Point(-2.,  1., 0.)));
		vids.push_back(mesh.add_vertex(Point( 2.,  1., 0.)));		
		vids.push_back(mesh.add_vertex(Point(-4., -2., 0.)));
		vids.push_back(mesh.add_vertex(Point( 4., -2., 0.)));
		vids.push_back(mesh.add_vertex(Point( 0., -2., 0.)));		

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

	~RefineThreeEdges() {
		// cleanup any pending stuff, but no exceptions allowed
	}

	// put in any custom data members that you need 
};





TEST_F(RefineThreeEdges, TestTwoEdgesToRefine)
{
	auto smooth_cost_property_map = mesh.add_property_map<edge_descriptor, double>("e:smooth_cost", 0.);

	auto he = mesh.halfedge(fids[1]);
	auto e0 = mesh.edge(he);
	auto e1 = mesh.edge(mesh.next(he));
	smooth_cost_property_map.first[e0] = 1.;
	smooth_cost_property_map.first[e1] = 1.;

	auto edges = Registration::getEdgesToRefine(mesh);
	EXPECT_EQ(edges.size(), 2);
	EXPECT_EQ(edges[0], e1);
	EXPECT_EQ(edges[1], e0);
}

TEST_F(RefineThreeEdges, TestTwoEdgesToSplit)
{
	auto smooth_cost_property_map = mesh.add_property_map<edge_descriptor, double>("e:smooth_cost", 0.);

	auto he = mesh.halfedge(fids[1]);
	auto e0 = mesh.edge(he);
	auto e1 = mesh.edge(mesh.next(he));
	auto e2 = mesh.edge(mesh.prev(he));
	smooth_cost_property_map.first[e0] = 1.;
	smooth_cost_property_map.first[e1] = 1.;

	auto edges = Registration::getEdgesToSplit(mesh);
	EXPECT_EQ(edges.size(), 3);
	EXPECT_EQ(edges[0], e1);
	EXPECT_EQ(edges[1], e0);
	EXPECT_EQ(edges[2], e2);
}
//
//TEST_F(RefineThreeEdges, TestRefine)
//{
//	auto smooth_cost_property_map = mesh.add_property_map<edge_descriptor, double>("e:smooth_cost", 0.);
//	auto he = mesh.halfedge(fids[0]);
//
//	auto e = mesh.edge(he);
//	smooth_cost_property_map.first[e] = 1.;
//
//	auto refined = Registration::refineDeformationGraph(mesh);
//
//	EXPECT_EQ(refined.number_of_faces(), 4);
//	EXPECT_EQ(refined.number_of_vertices(), 5);
//}
//
