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

double RegistrationError::mean() const
{
	return _mean;
}

double RegistrationError::variance() const
{
	return _variance;
}

double RegistrationError::median() const
{
	return _median;
}

double RegistrationError::min() const
{
	return _min;
}

double RegistrationError::max() const
{
	return _max;
}

RegistrationError::RegistrationError(std::vector<vertex_descriptor> v_ids, std::vector<double> errors)
	: _v_ids(v_ids)
	, _errors(errors)
{
	double sum = 0.;
	for (auto e : _errors)
	{
		if (e < _min)
			_min = e;
		if (e > _max)
			_max = e;
		sum += e;
	}
	if (!_errors.empty()) {
		_mean = sum / _errors.size();
		_variance = calculateVariance(_errors, _mean);
		_median = _errors[floor(_errors.size() / 2.)];
	}
}
	
}