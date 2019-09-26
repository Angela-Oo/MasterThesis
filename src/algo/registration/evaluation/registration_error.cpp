#include "registration_error.h"

namespace Registration {


double calculateVariance(const std::vector<double> & values, double mean)
{
	std::vector<double> diff(values.size());
	std::transform(values.begin(), values.end(), diff.begin(), [&mean](double x) { return x - mean; });
	double sq_sum = std::inner_product(diff.begin(), diff.end(), diff.begin(), 0.0);
	return std::sqrt(sq_sum / values.size());
}


size_t RegistrationError::size() const
{
	return _errors.size();
}

vertex_descriptor RegistrationError::v(size_t i) const
{
	return _v_ids[i];
}

double RegistrationError::error(size_t i) const
{
	return _errors[i];
}

const std::vector<double> & RegistrationError::errors()
{
	return _errors;
}

RegistrationError::RegistrationError(std::vector<vertex_descriptor> v_ids, std::vector<double> errors)
	: _v_ids(v_ids)
	, _errors(errors)
{ }




ErrorStatistics evalErrorStatistics(const std::vector<double> & errors)
{
	ErrorStatistics statistic;
	double sum = 0.;
	for (auto e : errors) {
		if (e < statistic.min)
			statistic.min = e;
		if (e > statistic.max)
			statistic.max = e;
		sum += e;
	}
	if (!errors.empty()) {
		statistic.mean = sum / errors.size();
		statistic.variance = calculateVariance(errors, statistic.mean);
		statistic.median = errors[floor(errors.size() / 2.)];
	}
	return statistic;
}
	
}