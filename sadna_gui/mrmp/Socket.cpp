#include "Socket.h"
#include <iostream>


int Socket::_num_of_sockets= 0;

void Socket::start() 
{
  if (!_num_of_sockets) 
  {
    WSADATA info;
    if (WSAStartup(MAKEWORD(2,0), &info)) 
    {
      std::cerr <<"Could not start WSA"<<std::endl;
      throw "Could not start WSA";
    }
  }
  ++_num_of_sockets;
}

void Socket::end() 
{
  WSACleanup();
}

Socket::Socket() : _s(0) 
{
  start();
  // UDP: use SOCK_DGRAM instead of SOCK_STREAM
  _s = socket(AF_INET, SOCK_STREAM, 0);

  if (_s == INVALID_SOCKET) 
  {
    std::cout <<"INVALID_SOCKET"<<std::endl;
    throw "INVALID_SOCKET";
  }

  _ref_counter = new int(1);
}

Socket::Socket(SOCKET s) : _s(s) 
{
  start();
  _ref_counter = new int(1);
};

Socket::~Socket() 
{
  if (! --(*_ref_counter)) 
  {
    close();
    delete _ref_counter;
  }

  --_num_of_sockets;
  if (!_num_of_sockets) 
    end();
}

Socket::Socket(const Socket& o) 
{
  _ref_counter=o._ref_counter;
  (*_ref_counter)++;
  _s =o._s;

  _num_of_sockets++;
}

Socket& Socket::operator=(Socket& o) 
{
  (*o._ref_counter)++;

  _ref_counter=o._ref_counter;
  _s          =o._s;

  _num_of_sockets++;

  return *this;
}

void Socket::close() 
{
  closesocket(_s);
}

std::string Socket::receive_line() 
{
  std::string ret;
  while (1) {
    char r;

    switch(recv(_s, &r, 1, 0)) 
    {
      case 0: // not connected anymore;
              // ... but last line sent
              // might not end in \n,
              // so return ret anyway.
        return ret;
      case -1:
        return "";
    }

    ret += r;
    if (r == '\n')  return ret;
  }
}

void Socket::send_line(std::string s) 
{
  s += '\n';
  send(_s, s.c_str(), s.length(), 0);
}

Socket_server::Socket_server(int port, int connections) 
{
  sockaddr_in sa;

  memset(&sa, 0, sizeof(sa));

  sa.sin_family = PF_INET;             
  sa.sin_port = htons(port);          
  _s = socket(AF_INET, SOCK_STREAM, 0);
  if (_s == INVALID_SOCKET) 
  {
    std::cout <<"INVALID_SOCKET"<<std::endl;
    throw "INVALID_SOCKET";
  }

  /* bind the socket to the internet address */
  if (bind(_s, (sockaddr *)&sa, sizeof(sockaddr_in)) == SOCKET_ERROR) 
  {
    closesocket(_s);
    std::cout <<"INVALID_SOCKET"<<std::endl;
    throw "INVALID_SOCKET";
  }
  
  listen(_s, connections);                               
}

Socket* Socket_server::accept() 
{
  SOCKET new_sock = ::accept(_s, 0, 0);
  if (new_sock == INVALID_SOCKET) 
  {
    int rc = WSAGetLastError();
    if(rc==WSAEWOULDBLOCK) 
    {
      return 0; // non-blocking call, no request pending
    }
    else 
    {
      std::cout <<"INVALID_SOCKET"<<std::endl;
      throw "Invalid Socket";
    }
  }

  Socket* r = new Socket(new_sock);
  return r;
}

Socket_client::Socket_client(const std::string& host, int port) : Socket() 
{
  std::string error;

  hostent *he;
  if ((he = gethostbyname(host.c_str())) == 0) 
  {
    std::cout <<"error"<<std::endl;
    exit(0);
  }

  sockaddr_in addr;
  addr.sin_family = AF_INET;
  addr.sin_port = htons(port);
  addr.sin_addr = *((in_addr *)he->h_addr);
  memset(&(addr.sin_zero), 0, 8); 

  if (::connect(_s, (sockaddr *) &addr, sizeof(sockaddr))) 
  {
    std::cout <<"error"<<std::endl;
    exit(0);
  }
}