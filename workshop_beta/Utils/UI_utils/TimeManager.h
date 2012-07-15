#ifndef TIME_MANAGER_H
#define TIME_MANAGER_H

class Time_manager
{
private:
  typedef std::pair<double,std::string>	Event;
  typedef std::vector<Event>			Event_vector;
  typedef Event_vector::iterator		Event_vector_iter;
public:
	Event_vector  time_log;
	CGAL::Timer   timer;
public:
	Time_manager()
	{
		time_log.clear();
		timer.start();
	}
	void write_time_log(std::string& expr)
	{
		time_log.push_back(Event(timer.time(),expr));
		#ifdef DEBUG_PRINT_TIME_LOG
        std::cout << "Time: "<< timer.time()<< "  Event: " << expr<< std::endl;
		#endif

		return;
	}
	void print_time_log()
	{
		Event_vector_iter curr,prev;
		for (curr = time_log.begin(); curr != time_log.end(); ++curr)
		{
			double duration (curr->first);
			if (curr != time_log.begin())
				duration -= prev->first;
            std::cout << "Time: "<< curr->first 
                      << "  Duration: " << duration 
                      << "  Event: " << curr->second 
                      << std::endl;
			prev = curr;
		}
		return;
	}
};

////////////////////
// Time manager   //
////////////////////
extern Time_manager global_tm;


#endif