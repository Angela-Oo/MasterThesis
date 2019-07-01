#pragma once
#include "file_writer.h"
#include <ceres/ceres.h>
#include <chrono>
#include <string>


std::string getDurationAsString(std::chrono::time_point<std::chrono::system_clock> start_time);

class CeresIterationLoggerGuard
{
	std::chrono::time_point<std::chrono::system_clock> _start_time;
	std::chrono::time_point<std::chrono::system_clock> _iteration_start_time;
	const ceres::Solver::Summary& _summary;
	size_t _iteration;
	std::shared_ptr<FileWriter> _logger;
public:
	long long get_time_in_ms();
	CeresIterationLoggerGuard(const ceres::Solver::Summary& summary, std::chrono::time_point<std::chrono::system_clock> start_time, size_t iteration = 0, std::shared_ptr<FileWriter> logger = nullptr);
	~CeresIterationLoggerGuard();
};

class CeresLogger
{
	std::chrono::time_point<std::chrono::system_clock> _start_time;
	std::chrono::time_point<std::chrono::system_clock> _end_time;
	size_t _iteration;
	std::shared_ptr<FileWriter> _logger;
public:
	void write(std::string text, bool log_time = true);
	CeresIterationLoggerGuard CreateCeresIterationLogger(const ceres::Solver::Summary& summary);
public:
	CeresLogger() {};
	CeresLogger(std::shared_ptr<FileWriter> logger);
	~CeresLogger();
};

