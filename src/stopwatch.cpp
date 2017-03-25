#include "stopwatch.hpp"

#include <time.h>
#include <stdio.h>

void StopWatch::start_timer() {
	gettimeofday(&tv, 0);
}

long StopWatch::elapsed_time() {
	struct timeval t2;
	int elapsedTime;
	gettimeofday(&t2, NULL);

	// elapsed time en mseg
	elapsedTime = (t2.tv_sec - tv.tv_sec) * 1000.0;      // s a ms
	elapsedTime += (t2.tv_usec - tv.tv_usec) / 1000.0;   // us a ms
                 
	return elapsedTime;

}

bool StopWatch::is_timeout(long mseconds) {
	
	return (elapsed_time() >= mseconds);
}
