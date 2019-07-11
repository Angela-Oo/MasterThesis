#include "stdafx.h"
#include "ceres_iteration_logger.h"


std::string getDurationAsString(std::chrono::time_point<std::chrono::system_clock> start_time)
{
	auto end_time = std::chrono::system_clock::now();
	auto duration_in_ms = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();

	double s = floor(static_cast<double>(duration_in_ms) / 1000.);
	double ms = duration_in_ms % 1000;

	std::stringstream ss;
	ss << "(s:ms) " << std::setfill('0') << s << ":" << std::setw(3) << ms;
	return ss.str();
}

std::string getDurationAsString_min_s_ms(std::chrono::time_point<std::chrono::system_clock> start_time)
{
	auto end_time = std::chrono::system_clock::now();
	auto duration_in_ms = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();

	double min = floor(static_cast<double>(duration_in_ms) / 60000.);
	double s = floor(static_cast<double>(duration_in_ms % 60000) / 1000.);	
	double ms = duration_in_ms % 1000;

	std::stringstream ss;
	ss << "(min:s:ms) " << std::setfill('0') << min << ":" << std::setw(2) << s << ":" << std::setw(3) << ms;
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
}


void CeresLogger::write(std::string text, bool log_time)
{
	std::stringstream ss;
	ss << std::endl;	
	if (log_time) {		
		ss << std::setprecision(4);
		ss << "time: " << std::setw(10) << getDurationAsString_min_s_ms(_start_time) << "   -   ";
	}
	ss << text;
	if (_logger)
		_logger->write(ss.str());
	std::cout << ss.str();
}

std::unique_ptr<CeresIterationLoggerGuard> CeresLogger::CreateCeresIterationLogger(const ceres::Solver::Summary& summary)
{
	auto iteration_logger = std::make_unique<CeresIterationLoggerGuard>(summary, _start_time, _iteration, _logger);
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
}
