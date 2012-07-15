#ifndef WRITTEN_PATHS_FILENAMES_H
#define WRITTEN_PATHS_FILENAMES_H

#include <boost/thread/mutex.hpp>
class Writen_paths_filenames
{
private:
  boost::mutex              mutex;
  std::vector<std::string>  writen_paths;
public:
  void push_paths_filename(std::string& filename)
  {
    mutex.lock();
    writen_paths.push_back(filename);
    mutex.unlock();
    return;
  }
  template <typename OutputIterator>
  void pop_paths_filenames(OutputIterator& oi)
  {
    mutex.lock();
    BOOST_FOREACH(std::string file_name, writen_paths)
      *oi++ = file_name;
    writen_paths.clear();
    mutex.unlock();
  }
}; //writen_paths_filenames

#endif //WRITTEN_PATHS_FILENAMES_H