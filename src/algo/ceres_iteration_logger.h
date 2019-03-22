#include <ceres/ceres.h>
#include <chrono>

class ICPLogIterationGuard
{
	std::chrono::time_point<std::chrono::system_clock> _start_time;
	const ceres::Solver::Summary& _summary;
	long long _total_time_in_ms;
	size_t _iteration;
public:
	long long get_time_in_ms();
	ICPLogIterationGuard(const ceres::Solver::Summary& summary, long long total_time_in_ms = 0, size_t iteration = 0);
	~ICPLogIterationGuard();
};
