 
#include <stdlib.h>
#include <iostream>

int main(int argc, char **argv)
{
std::string cmd;
cmd += "gnome-terminal --window -- ";
cmd += "bash -c \'source $HOME/catkin_ws/devel/setup.bash && ";
cmd += "$(rospack find perception_fusion)/../scripts/launch_telematics.sh";
cmd += "\'";
system(cmd.c_str());
return 0;
}
