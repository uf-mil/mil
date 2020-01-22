/*-----<>---------------------------------------------------------------
------------------------------------------------------------------------
Custom StopWatch Class
------------------------------------------------------------------------
----------------------------------------------------------------------*/
#ifndef STOPWATCH_H
#define STOPWATCH_H

// Built-in header for working with threads, needed by sleep_for
#include <thread>
// Built-in header for dealing with time
#include <chrono>

// Declare namespaces for the entire class definition
using namespace std::this_thread;
using namespace std::chrono;

/* ---<>-----------------------------------
StopWatch Class
---------------------------------------- */
class StopWatch
{
public:
  // Empty class constructor
  StopWatch()
  {
  }
  // Empty class destructor
  ~StopWatch()
  {
  }
  // Routine to start the stopwatch recording elapsed time
  void start()
  {
    startTime = system_clock::now();
  }
  // Function to return the current elapsed time but keeps
  // the stopwatch running
  double split()
  {
    endTime = system_clock::now();
    auto duration = duration_cast<milliseconds>(endTime - startTime);
    return (double)duration.count() / 1000.0;
  }
  // Fuction to have your entire program sleep for a specified amount of time
  void sleep(double seconds)
  {
    sleep_for(microseconds((unsigned int)(seconds * 1e6)));
  }

private:
  // Private class member variables
  system_clock::time_point startTime, endTime;
};

#endif  // STOPWATCH_H
