#ifndef STOPWATCH_HPP
#define STOPWATCH_HPP

#include <time.h>
#include <sys/time.h>

class StopWatch {
public:
	void start_timer();
	long elapsed_time();
	bool is_timeout(long mseconds);

private:
	struct timeval tv;

};

#endif /* STOPWATCH_HPP */
