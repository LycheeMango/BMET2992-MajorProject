#include <stdint.h>
#include "Arduino.h"

#ifndef SCHEDULER_H
#define SCHEDULER_H

#define MSEC 1000
#define TICK_1SEC 50

class Scheduler {
  
  private:
    unsigned long _interval; // in microseconds
    unsigned long _last_Tick_Time;
    
    void (*_tasks[10])(); // store all tasks that needs to be run, max 10 tasks (20ms tasks
    int _task_Count; // count of total tasks added to the scheduler (20ms tasks
    void (*_tasksSEC[10])(); // store all tasks that needs to be run every 1 sec, max 10 tasks
    int _task_Count_SEC; // count of total tasks added to the scheduler (1sec tasks
    uint16_t _tick_1_sec = 0;
    
    
  public:
    Scheduler(unsigned long interval=20); // in ms, default = 20ms)
    void initLastTick(); // initLastTick to current time;
    void addTask(void (*task)()); // add task to scheduler, 'queue', FIFO (20 ms tasks)
    void addTaskSEC(void(*task)()); // add task to scheduler (1 sec tasks)
    void run(); // run all tasks added to the tasks
  

};

#endif
