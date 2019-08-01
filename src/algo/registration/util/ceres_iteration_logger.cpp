#include "ceres_iteration_logger.h"
#include <sstream>
#include <iomanip>

std::string getDurationAsString(std::chrono::time_point<std::chrono::system_clock> start_time)
{
	auto end_time = std::chrono::system_clock::now();
	auto duration_in_ms = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();

	auto s = floor(static_cast<double>(duration_in_ms) / 1000.);
	auto ms = duration_in_ms % 1000;

	std::stringstream ss;
	ss << "(s:ms) " << std::setfill('0') << s << ":" << std::setw(3) << ms;
	return ss.str();
}

std::string getDurationAsString_min_s_ms(std::chrono::time_point<std::chrono::system_clock> start_time)
{
	auto end_time = std::chrono::system_clock::now();
	auto duration_in_ms = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();

	auto min = floor(static_cast<double>(duration_in_ms) / 60000.);
	auto s = floor(static_cast<double>(duration_in_ms % 60000) / 1000.);
	auto ms = duration_in_ms % 1000;

	std::stringstream ss;
	ss << "(min:s:ms) " << std::setfill('0') << min << ":" << std::setw(2) << s << ":" << std::setw(3) << ms;
	return ss.str();
}

std::string getDurationAsString_h_min_s_ms(std::chrono::time_point<std::chrono::system_clock> start_time)
{
	auto end_time = std::chrono::system_clock::now();
	auto duration_in_ms = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();

	auto h = floor(static_cast<double>(duration_in_ms) / 3600000.);
	auto min = floor(static_cast<double>(duration_in_ms % 3600000) / 60000.);
	auto s = floor(static_cast<double>(duration_in_ms % 60000) / 1000.);
	auto ms = duration_in_ms % 1000;

	std::stringstream ss;
	ss << "(h:min:s) " << std::setfill('0') << h << ":" << std::setw(2) << min << ":" << std::setw(2) << s;
	return ss.str();
}



void CeresIterationLoggerGuard::write(std::string text, bool log_time)
{
	_log_info += " " + text;
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
	ss << "  time iteration step: " << std::setw(10) << getDurationAsString_min_s_ms(_iteration_start_time);
	ss << "  time: " << std::setw(10) << getDurationAsString_h_min_s_ms(_start_time);
	ss << "  error: " << std::setw(8) << _summary.final_cost;
	ss << "  term: " << _summary.termination_type;
	ss << "  " << _log_info << " ";
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
		ss << "time: " << std::setw(10) << getDurationAsString_h_min_s_ms(_start_time) << "   -   ";
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
