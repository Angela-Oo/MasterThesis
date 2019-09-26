#pragma once

#include "mesh/mesh_definition.h"
#include <vector>

namespace Registration {

double calculateVariance(const std::vector<double> & values, double mean);

class RegistrationError
{
	std::vector<vertex_descriptor> _v_ids;
	std::vector<double> _errors;
public:
	size_t size() const;
	vertex_descriptor v(size_t i) const;
	double error(size_t i) const;
	const std::vector<double> & errors();
public:
	RegistrationError(std::vector<vertex_descriptor> v_ids, std::vector<double> errors);
};


class ErrorStatistics
{
public:
	double mean{ 0. };
	double variance{ 0. };
	double median{ 0. };
	double min{ INFINITY };
	double max{ 0. };
};

ErrorStatistics evalErrorStatistics(const std::vector<double> & errors);

}