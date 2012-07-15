#ifndef MANIFOLD_CONTAINER_BASE
#define MANIFOLD_CONTAINER_BASE

//-----------------------------------------------------
// a manifold contairer stores manifolds (pointers)
//-----------------------------------------------------

namespace mms{

template <typename Manifold>
class Manifold_container_base
{
public:
  typedef typename Manifold::Constraint  Constraint;
public:
  //constructor
  Manifold_container_base(): manifold_id(0) {};
  virtual int add_manifold(Manifold* manifold_ptr) = 0;
  virtual Manifold* get_manifold(int manifold_id)= 0;
  virtual Manifold* get_manifold(const Constraint& constraint)= 0; 
  virtual void clear()= 0;

  virtual int manifold_id_iterator_begin()= 0;
  virtual int manifold_id_iterator_next()= 0;
  virtual int manifold_id_iterator_end()= 0;
protected:
  int       manifold_id;
  int get_new_id()
  {
    return manifold_id++;
  }
};  //Manifold_container_base
} //namespace mms{
#endif // MANIFOLD_CONTAINER_BASE