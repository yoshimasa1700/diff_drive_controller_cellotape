#include <diff_drive_controller_cellotape.hpp>

int main(int argc, char **argv){
  ros::init(argc, argv, "diff_drive_controller_cellotape");

  ros::NodeHandle nh;

  DiffDriveController diff_drive_controller = DiffDriveController(nh);

  ros::spin();

  return 0;
}
