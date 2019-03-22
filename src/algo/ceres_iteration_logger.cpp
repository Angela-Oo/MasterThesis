#include "stdafx.h"
#include "ceres_iteration_logger.h"


ICPLogIterationGuard::ICPLogIterationGuard(const ceres::Solver::Summary& summary, long long total_time_in_ms, size_t iteration)
	: _summary(summary)
	, _total_time_in_ms(total_time_in_ms)
	, _iteration(iteration)
{
	_start_time = std::chrono::system_clock::now();

}

long long ICPLogIterationGuard::get_time_in_ms()
{
	auto end_time = std::chrono::system_clock::now();
	return std::chrono::duration_cast<std::chrono::milliseconds>(end_time - _start_time).count();
}

ICPLogIterationGuard::~ICPLogIterationGuard()
{
	std::cout << "Final report:\n" << _summary.BriefReport() << std::endl;
	auto elapse = get_time_in_ms();
	_total_time_in_ms += elapse;

	auto time_to_string = [](long long time_in_ms) {
		long long s = floor(static_cast<double>(time_in_ms) / 1000.);
		long long ms = time_in_ms - (s * 1000);
		return std::to_string(s) + "s " + std::to_string(ms) + "ms";
	};


	std::cout << "Ceres Solver Iteration: " << _iteration << " sub iterations: " << _summary.num_inner_iteration_steps << ", Duration " << time_to_string(elapse) << ", Total time: " << time_to_string(_total_time_in_ms)
		<< ", Initial cost: " << _summary.initial_cost << ", Final cost: " << _summary.final_cost << ", Termination: " << _summary.termination_type << std::endl << std::endl;
}

