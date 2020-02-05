#include "../include/follower.h"
#include "follower_library.cpp"

int main(int argc, char** argv)
{
  init(argc, argv, "follower", init_options::NoSigintHandler);
  NodeHandle n;
  ROS_INFO_STREAM("Demo Node Is Up");
  Subscriber imageSubRGB = n.subscribe("/camera/color/image_raw", 1, imageCallbackRGB);
  Subscriber imageSubIR = n.subscribe("/camera/infra1/image_rect_raw", 1, imageCallbackIR);

  signal(SIGINT, SigintHandler);
  Rate rate(30); // 30Hz

  while (ok())
  {

    rate.sleep();
    spinOnce();
  }
  return 0;
}
