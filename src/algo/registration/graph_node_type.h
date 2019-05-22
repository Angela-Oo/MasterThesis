#pragma once
#include "boost/graph/adjacency_list.hpp"

struct node_t {
	typedef boost::vertex_property_tag kind;
};

struct edge_t {
	typedef boost::edge_property_tag kind;
};

typedef boost::adjacency_list<>::vertex_descriptor vertex_index;
typedef boost::adjacency_list<>::edge_descriptor edge_index;