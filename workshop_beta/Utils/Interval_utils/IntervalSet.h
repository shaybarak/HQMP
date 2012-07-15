#ifndef Interval_set_H  
#define Interval_set_H  

#include "Utils\Interval_utils\Interval.h"

template <class NT, typename Convertor = Default_convertor<NT> >
class Interval_set
{
public:
  typedef typename Interval<NT, Convertor>  Interval;
  typedef std::list<Interval>               Interval_list;
  typedef typename Interval_list::iterator  Interval_list_iter;
private:
  typedef Interval_set <NT, Convertor>      Self;
public:
  Interval_set(bool all_is_legal)
  {
    if (all_is_legal)
    {
      intervals.push_front(Interval());
    }
  }
  void insert_or (const Interval& i)
  {
    assert(false);
    if (is_unbounded())
      return;
    if (intervals.empty())
      return intervals.push_front(i);
    if (i.is_unbounded())
    {
      intervals.clear();
      intervals.push_front(i);
      return ;
    }
    if (i.is_lower_bounded() == false)
      return insert_or_lower_unbounded(i);
    if (i.is_higher_bounded() == false)
      return insert_or_higher_unbounded(i);

    //i is unbounded
    return insert_or_bounded(i);
  }

  void insert_and (const Interval& i)
  {
    if (is_unbounded())
    {
	    intervals.clear();
	    intervals.push_front(i);
	    return;
    }
    if (intervals.empty())
	    return;
    if (i.is_unbounded())
    {
      //this may be a bug
      //check if is used
      assert(false);
    
      CGAL_postcondition (false);
      //old version
      intervals.clear();
      return;
      //new version
      return;
    }
    if (i.is_lower_bounded() == false)
    {
      insert_and_lower_unbounded(i);
      return;
    }
    if (i.is_higher_bounded() == false)
    {
      insert_and_higher_unbounded(i);
      return;
    }
    //i is unbounded
    insert_and_bounded(i);
    return;	
  }
  void insert_and_complement (const Interval& i)
  {
    CGAL_precondition (i.is_bounded());
    Interval i1 (i.lower(), true, false);
    Interval i2 (i.higher(), false, true);
    Self tmp_intervals1(true);
    Self tmp_intervals2(true);

    tmp_intervals1.assign_interval_list(intervals.begin(),intervals.end());
    tmp_intervals2.assign_interval_list(intervals.begin(),intervals.end());
    
    tmp_intervals1.insert_and(i1);
    tmp_intervals2.insert_and(i2);

    intervals.clear();
    intervals.assign(tmp_intervals1.intervals.begin(),tmp_intervals1.intervals.end());
    intervals.insert(intervals.end(),tmp_intervals2.intervals.begin(),tmp_intervals2.intervals.end());

    return;
  }
  bool is_unbounded() const
  {
    if (intervals.size() == 1 ) 
      return intervals.front().is_unbounded();
    return false;
  }
  bool contains(const NT& val)
  {
    for (Interval_list_iter it (intervals.begin()); it!= intervals.end(); ++it)
    {
      if (it->contains(val) == true)
        return true;
    }
    return false;
  }
  void print_nice()
  {
    Interval_list_iter it;
    cout << " { ";
    for (it = intervals.begin(); it != intervals.end() ; )
    {
      it->print_nice();
      ++it;
      if (it != intervals.end())
        cout << " , ";
    }
    cout << " } " <<endl;
    return;
  }

  Interval_list& get_intervals()
  {
    return intervals;
  }
  const Interval_list& get_intervals() const 
  {
    return intervals;
  }
  void assign_interval_list(const Interval_list& new_intervals)
  {    
    intervals.assign(new_intervals.begin(),new_intervals.end());
    return;
  }
  template <typename InputIterator>
  void assign_interval_list(InputIterator& begin, InputIterator& end)
  {    
    intervals.assign(begin, end);
    return;
  }
private:
  void insert_or_lower_unbounded(const Interval& i)
  {
    CGAL_precondition (i.is_lower_bounded() == false);
    if (intervals.empty())
    {
      intervals.push_front(i);
      return;
    }
    if (intervals.front().is_lower_bounded() == false )
    {
      if (CGAL::compare (i.higher(),intervals.front().higher()) == CGAL::SMALLER)
        return;
      else
      {	//first element has no effect, remove it
        intervals.pop_front();
        if (intervals.empty())
        {
          intervals.push_front(i);
          return;
        }
      }
    }

    if (intervals.back().is_higher_bounded() == false )
    {
      if (CGAL::compare (i.higher() , intervals.back().lower()) == CGAL::LARGER)
      {
        intervals.clear();
        intervals.push_back(Interval());
        return;
      }
    }

	//at this point the first elemt is bounded 
	//and if the last is unbounded we will not
	//reach it
    for (Interval_list_iter it (intervals.begin()); it != intervals.end(); ++ it)
    {				
      if (CGAL::compare (it->lower() , i.higher()) == CGAL::LARGER)
      {
        intervals.erase(intervals.begin(),it);
        intervals.push_front(i);
        return;
      }
      if (CGAL::compare (it->higher() , i.higher()) == CGAL::LARGER)
      {
        Interval tmp (it->higher(),true,false);
        ++it;
        intervals.erase(intervals.begin(),it);
        intervals.push_front(tmp);
        return;
      }
    }
    
    //if we reached here then the interval should be i
    intervals.clear();
    intervals.push_front(i);
    return;
  }

  void insert_or_higher_unbounded(const Interval& i)
  {
    CGAL_precondition (i.is_higher_bounded() == false);
    if (intervals.empty())
    {
      intervals.push_front(i);
      return;
    }
    if (intervals.front().is_lower_bounded() == false )
    {
      if (CGAL::compare (i.lower() , intervals.front().higher()) == CGAL::SMALLER)
      {
        intervals.clear();
        intervals.push_back(Interval());
        return;
      }
    }
    if (intervals.back().is_higher_bounded() == false )
    {
      if (CGAL::compare (i.lower(),intervals.back().lower()) == CGAL::LARGER)
        return;
      else
      {
        //lastelement has no effect, remove it
        intervals.pop_back();
        if (intervals.empty())
        {
          intervals.push_front(i);
          return;
        }
      }
    }
    //at this point if the first elemt is unbounded it has no effect
    //and the last is unbounded
    Interval_list_iter it=intervals.end();
    for ( ; it != intervals.begin(); )
    {	
      --it;
      if (CGAL::compare (it->higher(),i.lower()) == CGAL::SMALLER)
      {
        ++it;
        intervals.erase(it,intervals.end());
        intervals.push_back(i);
        return;
      }
      if (CGAL::compare(it->lower() , i.lower()) == CGAL::SMALLER)
      {
        Interval tmp (it->lower(),false,true);
        intervals.erase(it,intervals.end());
        intervals.push_back(tmp);
        return;
      }
    }
    //if we reached here then the interval should be i
    intervals.clear();
    intervals.push_front(i);
    return;
  }
  void insert_or_bounded(const Interval& i)
  {
    CGAL_precondition (i.is_bounded());
    
    Interval tmp1(i.lower(),true,false);
    Interval tmp2(i.higher(),false,true);
    
    Self          tmp_interval_set_1(true);
    Interval_list tmp_interval_list1 = tmp_interval_set_1.get_intervals();

    Self          tmp_interval_set_2(true);
    Interval_list tmp_interval_list2 = tmp_interval_set_2.get_intervals();

    tmp_interval_list1.assign(intervals.begin(),intervals.end());
    tmp_interval_list2.assign(intervals.begin(),intervals.end());
    
    tmp_interval_set_1.insert_and_lower_unbounded(tmp1);
    tmp_interval_set_2.insert_and_lower_unbounded(tmp2);
    if (tmp_interval_list1.empty())
    {
      tmp_interval_list1.push_back(i);
    }
    else
    {
      if (CGAL::compare (tmp_interval_list1.back().higher() ,
                         i.lower()) == CGAL::EQUAL)
        tmp_interval_list1.back().set_higher(i.higher());
      else
        tmp_interval_list1.push_back(i);
    }

    if (tmp_interval_list2.empty())
    {
      intervals.assign(tmp_interval_list1.begin(),tmp_interval_list1.end());
    }
    else
    {
      if (CGAL::compare ( tmp_interval_list2.front().lower() , 
                          tmp_interval_list1.back().higher()) == CGAL::EQUAL)
      {
        //we need to unify the last element of 
        //tmp_intervals1 with the first of tmp_intervals2
        if (tmp_interval_list1.back().is_lower_bounded() == false)
        {
          tmp_interval_list2.front().set_lower_unbounded();
          intervals.clear();
          intervals.insert(intervals.begin(),tmp_interval_list2.begin(),tmp_interval_list2.end());
          return;
        }
        else if (tmp_interval_list2.front().is_higher_bounded() == false)
        {
          tmp_interval_list1.back().set_higher_unbounded();
          intervals.assign(tmp_interval_list1.begin(),tmp_interval_list1.end());
          return;
        }				
        else
        {
          tmp_interval_list1.back().set_higher(tmp_interval_list2.front().higher());
          tmp_interval_list2.pop_front();
          intervals.assign(tmp_interval_list1.begin(),tmp_interval_list1.end());
          intervals.insert(intervals.end(),tmp_interval_list2.begin(),tmp_interval_list2.end());
          return;
        }
      }
      else
      {
        intervals.assign(tmp_interval_list1.begin(),tmp_interval_list1.end());
        intervals.insert(intervals.end(),tmp_interval_list2.begin(),tmp_interval_list2.end());
      }
    }
    return;
  }
  void insert_and_lower_unbounded (const Interval& i)
  {
    CGAL_precondition (i.is_lower_bounded() == false);
    if (intervals.empty())
      return;
    if (intervals.back().is_higher_bounded() == false )
    {
      //highest value can be i.highest
      if (CGAL::compare (i.higher() , intervals.back().lower()) == CGAL::SMALLER)
      {
        intervals.pop_back();
        if (intervals.empty())
          return;
      }
      else
      {
        Interval tmp(intervals.back().lower(), i.higher());
        intervals.pop_back();
        intervals.push_back(tmp);
      }
    }
    if (intervals.front().is_lower_bounded() == false )	
    {
      if (CGAL::compare (i.higher() , intervals.front().higher()) == CGAL::SMALLER)
      {
        intervals.clear();
        intervals.push_front(i);
        return;
      }
    }
    else
    {
      //just take care of first element for the loop
      if (CGAL::compare (intervals.front().lower() , i.higher()) == CGAL::LARGER)
      {
        intervals.clear();
        //???
        //intervals.push_front(i);
        return;
      }
      if (CGAL::compare (intervals.front().higher() ,  i.higher()) == CGAL::LARGER)
      {
        Interval tmp(intervals.front().lower(), i.higher());
        intervals.clear();
        intervals.push_front(tmp);
        return;
      }
    }
    Interval_list_iter it= intervals.begin();
    ++it;
    for ( ; it != intervals.end(); ++ it)
    {				
      if (CGAL::compare (it->lower() , i.higher()) == CGAL::LARGER )
      {
        intervals.erase(it,intervals.end());
        return;
      }
      if (CGAL::compare (it->higher() , i.higher()) == CGAL::LARGER)
      {
        Interval tmp(it->lower(), i.higher());
        intervals.erase(it,intervals.end());
        intervals.push_back(tmp);
        return;
      }
    }
    //intervals.clear();
    return;
  }
  void insert_and_higher_unbounded (const Interval& i)
  {
    CGAL_precondition (i.is_higher_bounded() == false);
    if (intervals.empty())
      return;
    if (intervals.back().is_higher_bounded() == false )
    {
      if (CGAL::compare (i.lower() ,  intervals.back().lower()) == CGAL::LARGER)
      {
        intervals.clear();
        intervals.push_front(i);
        return;
      }
    }
    if (intervals.front().is_lower_bounded() == false )	
    {
      if (CGAL::compare (i.lower() , intervals.front().higher()) == CGAL::LARGER)
      {
        intervals.pop_front();
        if (intervals.empty())
          return;
      }
      else
      {
        Interval tmp( i.lower(),intervals.front().higher());
        intervals.clear();
        intervals.push_front(tmp);
        return;
      }
    }
    else
    {
      //just take care of first element for the loop
      if (CGAL::compare (intervals.front().lower() , i.lower()) == CGAL::LARGER )
      {
        return;
      }
      if (CGAL::compare (intervals.front().higher() , i.lower()) == CGAL::LARGER )
      {
        Interval tmp( i.lower(),intervals.front().higher());
        intervals.pop_front();
        intervals.push_front(tmp);
        return;
      }
    }
    Interval_list_iter it= intervals.begin();
    //++it;
    for ( ; it != intervals.end(); ++ it)
    {				
      if (CGAL::compare(it->lower() , i.lower()) == CGAL::LARGER)
      {
        intervals.erase(intervals.begin(),it);
        return;
      }
      if (CGAL::compare(it->higher() , i.lower()) == CGAL::LARGER)
      {
        Interval tmp(i.lower(),it->higher());
        ++it;
        intervals.erase(intervals.begin(),it);
        intervals.push_front(tmp);
        return;
      }
    }
    intervals.clear();
    return;
  }
  void insert_and_bounded(const Interval& i)
	{
      CGAL_precondition (i.is_bounded());
      
      Interval tmp1(i.higher(),true,false);
      Interval tmp2(i.lower(),false,true);
      
      insert_and_lower_unbounded(tmp1);
      insert_and_higher_unbounded(tmp2);
      return;
	}

private:
	//every interval is a legal partioining of the line
	Interval_list intervals;
};
#endif //Interval_set_H  