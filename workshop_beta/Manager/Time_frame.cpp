#include "stdafx.h"
#include "Time_frame.h"

void Time_frame::start()
{
  timer.start();
}
void Time_frame::change_color(double duration_)
{
  CGAL_precondition (duration_ > 0);
  mutex.lock();
  interval_id++;
  duration = duration_;
  timer.reset();

  switch(curr_color)
  {
  case RED:
  case BLUE:
    {
      prev_color = curr_color;
      curr_color = GRAY;
      break;
    }
  case GRAY:
    {
      curr_color = (prev_color == RED) ? BLUE : RED;
      prev_color = GRAY;
      break;
    }
  }    
  mutex.unlock();
  return;
}

double Time_frame::remaining_time() 
{
  mutex.lock();
  double t = duration - timer.time();
  if (t <0)
    std::cout <<"t: "<<t <<" "
              <<"duration " << duration<<" "
              <<"timer.time() " << timer.time()<<std::endl;

  CGAL_postcondition(t>=0);
  mutex.unlock();
  return t;
}

int Time_frame::id() const 
{
  return interval_id;
}