#ifndef SOCKET_H
#define SOCKET_H

#include <WinSock2.h>
#include <winsock2.h>
#include <windows.h>
#include <fcntl.h>
#include <string>

////////////////////////////////////////////////////
//      Socket
////////////////////////////////////////////////////
class Socket 
{
public: 
  virtual ~Socket();          //dtr
  Socket(const Socket&);      //copy ctr
  Socket& operator=(Socket&); //assignment
  
public: //general functions
  void   close();
  
public: //recieving functions  
  std::string receive_line();
public: //sending functions
  // The parameter of SendLine is not a const reference
  // because send_line modifes the std::string passed.
  void   send_line (std::string);


protected:
  friend class Socket_server;

  Socket(SOCKET s); //ctr
  Socket();         //ctr


  SOCKET _s;
  int* _ref_counter;

private:
  static void start();
  static void end();
  static int  _num_of_sockets;
}; //Socket


////////////////////////////////////////////////////
//      Socket_client
////////////////////////////////////////////////////
class Socket_client : public Socket 
{
public:
  Socket_client(const std::string& host, int port);
}; //Socket_client


////////////////////////////////////////////////////
//      Socket_server
////////////////////////////////////////////////////
class Socket_server : public Socket 
{
public:
  Socket_server(int port, int connections);

  Socket* accept();
}; //Socket_server

#endif //SOCKET_H