#include "stdafx.h"
#include "ceres_iteration_logger.h"


std::string getDurationAsString(std::chrono::time_point<std::chrono::system_clock> start_time)
{
	auto end_time = std::chrono::system_clock::now();
	auto duration_in_ms = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();

	double s = floor(static_cast<double>(duration_in_ms) / 1000.);
	double ms = duration_in_ms - (s * 1000);
	std::stringstream ss;
	ss << std::setw(4) << s << " s " << std::setw(3) << ms << " ms";
	return ss.str();
}

long long CeresIterationLoggerGuard::get_time_in_ms()
{
	auto end_time = std::chrono::system_clock::now();
	return std::chrono::duration_cast<std::chrono::milliseconds>(end_time - _start_time).count();
}

CeresIterationLoggerGuard::CeresIterationLoggerGuard(const ceres::Solver::Summary& summary, 
													 std::chrono::time_point<std::chrono::system_clock> start_time,
													 size_t iteration,
													 std::shared_ptr<FileWriter> logger)
	: _summary(summary)
	, _start_time(start_time)
	, _iteration(iteration)
	, _logger(logger)
{
	_iteration_start_time = std::chrono::system_clock::now();
}

CeresIterationLoggerGuard::~CeresIterationLoggerGuard()
{
	std::stringstream ss;
	ss << std::setprecision(4);
	ss << std::endl << "Iteration: " << std::setw(3) << _iteration << "  steps: " << std::setw(3) << _summary.iterations.size();
	ss << "  time iteration step: " << std::setw(10) << getDurationAsString(_iteration_start_time);
	ss << "  time: " << std::setw(10) << getDurationAsString(_start_time);
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




CeresIterationLoggerGuard CeresLogger::CreateCeresIterationLogger(const ceres::Solver::Summary& summary)
{
	auto iteration_logger = CeresIterationLoggerGuard(summary, _start_time, _iteration, _logger);
	_iteration++;
	return iteration_logger;
}

CeresLogger::CeresLogger(std::shared_ptr<FileWriter> logger)
	: _logger(logger)
	, _start_time(std::chrono::system_clock::now())
	, _iteration(0)
{

}

CeresLogger::~CeresLogger()
{
	std::stringstream ss;
	ss << std::setprecision(4);
	ss << std::endl << "Total Time: " << std::setw(10) << getDurationAsString(_start_time);

	if (_logger)
		_logger->write(ss.str());
	std::cout << ss.str();
}
