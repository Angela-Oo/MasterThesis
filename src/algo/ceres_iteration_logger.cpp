#include "stdafx.h"
#include "ceres_iteration_logger.h"

long long CeresIterationLoggerGuard::get_time_in_ms()
{
	auto end_time = std::chrono::system_clock::now();
	return std::chrono::duration_cast<std::chrono::milliseconds>(end_time - _start_time).count();
}

CeresIterationLoggerGuard::CeresIterationLoggerGuard(const ceres::Solver::Summary& summary, long long total_time_in_ms, size_t iteration, std::shared_ptr<FileWriter> logger)
	: _summary(summary)
	, _total_time_in_ms(total_time_in_ms)
	, _iteration(iteration)
	, _logger(logger)
{
	_start_time = std::chrono::system_clock::now();

}

CeresIterationLoggerGuard::~CeresIterationLoggerGuard()
{
	auto elapse = get_time_in_ms();
	_total_time_in_ms += elapse;

	auto time_to_string = [](double time_in_ms) {
		double s = floor(static_cast<double>(time_in_ms) / 1000.);
		double ms = time_in_ms - (s * 1000);
		std::stringstream ss;
		ss << std::setw(4) << s << " s " << std::setw(3) << ms << " ms";
		return ss.str();
	};

	std::stringstream ss;
	ss << std::setprecision(4);
	ss << std::endl << "Iteration: " << std::setw(3) << _iteration << "  steps: " << std::setw(3) << _summary.iterations.size();
	ss << "  time: " << std::setw(10) << time_to_string(_total_time_in_ms);
	ss << "  error: " << std::setw(8) << _summary.final_cost;
	ss << "  term: " << _summary.termination_type << "  ";

	if(_logger)
		_logger->write(ss.str());
	std::cout << ss.str();
	//std::cout << std::setprecision(4);
	//std::cout << std::endl << "Iteration: " << std::setw(3) << _iteration << "  steps: " << std::setw(3) << _summary.iterations.size();	
	////std::cout << " duration " << time_to_string(elapse);
	//std::cout << "  time: " << std::setw(10) << time_to_string(_total_time_in_ms);
	////std::cout << "Initial cost: " << _summary.initial_cost;
	//std::cout << "  error: " << std::setw(6) << _summary.final_cost;
	////if (_summary.termination_type != 1)
	//std::cout << "  term: " << _summary.termination_type;
	//std::cout << "  ";
}

