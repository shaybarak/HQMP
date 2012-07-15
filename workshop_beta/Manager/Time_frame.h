#ifndef TIME_FRAME_H
#define TIME_FRAME_H

#include <boost/thread/mutex.hpp>

class Time_frame
{
public:
  enum Interval_color {RED, BLUE, GRAY}; 
private:
  boost::mutex    mutex;				// The mutex to synchronise the data
  Interval_color  curr_color;
  Interval_color  prev_color;
  int             interval_id;

  double      duration;
  CGAL::Timer timer;
public:
  Time_frame(Interval_color color_, double duration_)
    : curr_color(color_), prev_color(GRAY), duration(duration_), interval_id(0)
  {
    assert(curr_color != GRAY);    
  }
public:
  void start();
  void change_color(double duration_);
  Interval_color get_color() const {return curr_color;} 
  double remaining_time();
  int id() const;

}; //Time_frame


#define PLAYER_A_COLOR      Time_frame::RED;
#define PLAYER_B_COLOR      Time_frame::BLUE;
//typedef Time_frame::RED     PLAYER_A_COLOR;
//typedef Time_frame::BLUE    PLAYER_B_COLOR;
#endif //TIME_FRAME_H