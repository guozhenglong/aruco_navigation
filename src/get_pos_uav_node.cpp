#include "localization.h"

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "get_nav_data_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    Localization::local_position Get_Pos_UAV(nh, pnh);
    return 0;
}