#include "scheduler.h"

Scheduler::Scheduler(unsigned long interval) {
  _interval = interval*MSEC;
  _last_Tick_Time = 0;
  _task_Count = 0;
}

void Scheduler::initLastTick() {
  _last_Tick_Time = micros();  
}

void Scheduler::addTask(void (*task)()) {
  if (_task_Count < 10) {
    _tasks[_task_Count] = task;
    _task_Count++;
  }
}

void Scheduler::addTaskSEC(void (*task)()) {
  if (_task_Count_SEC < 10) {
    _tasksSEC[_task_Count_SEC] = task;
    _task_Count_SEC++;
  }
}

void Scheduler::run() {
  unsigned long current_Time = micros();
  if (current_Time - _last_Tick_Time >= _interval) {
    _last_Tick_Time = current_Time;
    //>>> 20 msec tasks go here
    for (int i = 0; i < _task_Count; i++) {
      _tasks[i](); // Execute the task added by addTask() []
    }
    // After executing the task inserted by addTask
    // it can also run these hard coded tasks

    _tick_1_sec++;
  }
  if (_tick_1_sec >= TICK_1SEC) {
    _tick_1_sec = 0;

    // >>> 1 sec tasks go here
    for (int i = 0; i < _task_Count_SEC; i++) {
      _tasksSEC[i](); // Execute the task added by addTask() []
    }
  
  }

  
}
