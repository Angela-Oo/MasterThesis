#pragma once
#include "file_writer.h"
#include <ceres/ceres.h>
#include <chrono>

class CeresIterationLoggerGuard
{
	std::chrono::time_point<std::chrono::system_clock> _start_time;
	const ceres::Solver::Summary& _summary;
	long long _total_time_in_ms;
	size_t _iteration;
	std::shared_ptr<FileWriter> _logger;
public:
	long long get_time_in_ms();
	CeresIterationLoggerGuard(const ceres::Solver::Summary& summary, long long total_time_in_ms = 0, size_t iteration = 0, std::shared_ptr<FileWriter> logger = nullptr);
	~CeresIterationLoggerGuard();
};
