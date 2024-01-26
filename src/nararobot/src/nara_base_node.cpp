#include "nara_base.h"

int main(int argc, char** argv )
{
    ros::init(argc, argv, "nara_base_node");
    NaraBase nara;
    ros::spin();
    return 0;
}
