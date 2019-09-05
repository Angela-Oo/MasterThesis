#pragma once

#include "mesh/mesh_definition.h"
#include <vector>

namespace Registration {

double calculateVariance(const std::vector<double> & values, double mean);

class RegistrationError
{
private:
	double _mean{ 0. };
	double _variance{ 0. };
	double _median{ 0. };
	double _min{ INFINITY };
	double _max{ 0. };
	std::vector<vertex_descriptor> _v_ids;
	std::vector<double> _errors;
public:
	size_t size() const;
	vertex_descriptor v(size_t i) const;
	double error(size_t i) const;
	double mean() const;
	double variance() const;
	double median() const;
	double min() const;
	double max() const;
public:
	RegistrationError(std::vector<vertex_descriptor> v_ids, std::vector<double> errors);
};

}