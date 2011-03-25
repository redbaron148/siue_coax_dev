/*
 * teleop_base_keyboard
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the <ORGANIZATION> nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <termios.h>
#include <signal.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/poll.h>

#include <boost/thread/thread.hpp>
#include <ros/ros.h>
#include <coax_client/Keyboard.h>

// should we continuously send commands?
bool always_command = true;

class TBK_Node
{
  private:
    coax_client::Keyboard keys_;
    ros::NodeHandle n_;
    ros::Publisher pub_;

  public:
    TBK_Node()
    {
      pub_ = n_.advertise<coax_client::Keyboard>("keyboard",1);
    }
    ~TBK_Node() { }
    void keyboardLoop();
};

TBK_Node* tbk;
int kfd = 0;
struct termios cooked, raw;
bool done;

int
main(int argc, char** argv)
{
  ros::init(argc,argv,"tbk", ros::init_options::AnonymousName | ros::init_options::NoSigintHandler);
  TBK_Node tbk;

  boost::thread t = boost::thread::thread(boost::bind(&TBK_Node::keyboardLoop, &tbk));
  
  ros::spin();

  t.interrupt();
  t.join();
  //tbk.stopRobot();
  tcsetattr(kfd, TCSANOW, &cooked);

  return(0);
}

void
TBK_Node::keyboardLoop()
{
  char c = ' ';
  //double max_tv = max_speed;
  //double max_rv = max_turn;
  //bool dirty=false;
  coax_client::Keyboard key;

  //int speed=0;
  //int turn=0;

  // get the console in raw mode
  tcgetattr(kfd, &cooked);
  memcpy(&raw, &cooked, sizeof(struct termios));
  raw.c_lflag &=~ (ICANON | ECHO);
  raw.c_cc[VEOL] = 1;
  raw.c_cc[VEOF] = 2;
  tcsetattr(kfd, TCSANOW, &raw);

  puts("Reading from keyboard");

  struct pollfd ufd;
  ufd.fd = kfd;
  ufd.events = POLLIN;
  for(;;)
  {
    boost::this_thread::interruption_point();
    
    // get the next event from the keyboard
    int num;
    if((num = poll(&ufd, 1, 250)) < 0)
    {
      perror("poll():");
      return;
    }
    else if(num > 0)
    {
      if(read(kfd, &c, 1) < 0)
      {
        perror("read():");
        return;
      }
    }
    else
      continue;
      
    key.key = c;

    pub_.publish(key);
  }
}
