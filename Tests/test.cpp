#include "pch.h"

#include "mLibInclude.h"
#include "algo/surface_mesh/mesh_definition.h"
#include "algo/registration/deformation_graph/refine_deformation_graph.h"


TEST(TestCaseName, TestName) {
  EXPECT_EQ(1, 1);
  EXPECT_TRUE(true);
}

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


TEST_F(RefineOneEdge, TestMesh) 
{	
	EXPECT_EQ(mesh.number_of_faces(), 2);
	EXPECT_EQ(mesh.number_of_vertices(), 4);

	auto smooth_cost_property_map = mesh.add_property_map<edge_descriptor, double>("e:smooth_cost", 0.);
	auto he = mesh.halfedge(fids[0]);

	auto s = mesh.source(he);
	auto t = mesh.target(he);
	EXPECT_EQ(vids[2], s);
	EXPECT_EQ(vids[0], t);
}

TEST_F(RefineOneEdge, TestSplitCenterEdge)
{
	auto he = mesh.halfedge(fids[0]);
	auto e = mesh.edge(he);

	Registration::splitEdge(e, mesh);

	EXPECT_EQ(mesh.number_of_faces(), 4);
	EXPECT_EQ(mesh.number_of_vertices(), 5);

	auto f_it = mesh.faces().begin();
	auto last_vid = *(mesh.vertices().begin() + 4);

	auto vf0 = mesh.vertices_around_face(mesh.halfedge(*f_it)).begin();
	EXPECT_EQ(*vf0, vids[1]);
	vf0++;
	EXPECT_EQ(*vf0, vids[2]);
	vf0++;
	EXPECT_EQ(*vf0, last_vid);

	auto vf1 = mesh.vertices_around_face(mesh.halfedge(*(++f_it))).begin();
	EXPECT_EQ(*vf1, vids[3]);
	vf1++;
	EXPECT_EQ(*vf1, vids[0]);
	vf1++;;
	EXPECT_EQ(*vf1, last_vid);

	auto vf2 = mesh.vertices_around_face(mesh.halfedge(*(++f_it))).begin();
	EXPECT_EQ(*vf2, last_vid);
	vf2++;
	EXPECT_EQ(*vf2, vids[0]);
	vf2++;;
	EXPECT_EQ(*vf2, vids[1]);

	auto vf3 = mesh.vertices_around_face(mesh.halfedge(*(++f_it))).begin();
	EXPECT_EQ(*vf3, last_vid);
	vf3++;
	EXPECT_EQ(*vf3, vids[2]);
	vf3++;;
	EXPECT_EQ(*vf3, vids[3]);
}

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

TEST_F(RefineOneEdge, TestRefine)
{
	auto smooth_cost_property_map = mesh.add_property_map<edge_descriptor, double>("e:smooth_cost", 0.);
	auto he = mesh.halfedge(fids[0]);

	auto e = mesh.edge(he);
	smooth_cost_property_map.first[e] = 1.;

	auto refined = Registration::refineDeformationGraph(mesh);

	EXPECT_EQ(refined.number_of_faces(), 4);
	EXPECT_EQ(refined.number_of_vertices(), 5);
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