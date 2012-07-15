#ifndef SAFE_TYPE_H
#define SAFE_TYPE_H

//#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>


template <typename T>
class Safe_type
{
private:
  T             t;
  boost::mutex  mutex;
public:
  Safe_type(const T& t_) :t(t_)
  {}
  
  void set(const T& t_)
  {
    mutex.lock();
    t = t_;
    mutex.unlock();

    return;
  }  
  T get() const 
  {
    return t;
  }
}; //Safe_type
#endif //SAFE_TYPE_H