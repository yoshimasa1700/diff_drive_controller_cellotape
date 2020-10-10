#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

#include <pigpiod_if2.h>

class DiffDriveController{
 public:
  DiffDriveController(ros::NodeHandle &nh){
    // register subscriber.
    sub_command = nh.subscribe("cmd_vel", 1000, &DiffDriveController::command_callback, this);
    timer = nh.createTimer(ros::Duration(0.1), &DiffDriveController::timer_callback, this);

    init_pigpio();
    setup_gpio_pins();
  }

  ~DiffDriveController(){
    motor_idle();
    pigpio_stop(pi);
  }

private:
  ros::Timer timer;
  int pi;
  ros::Subscriber sub_command;

  geometry_msgs::Twist command_;

  void timer_callback(const ros::TimerEvent& e){
    // gpio_write(pi, 18, 1);
    // gpio_write(pi, 19, 1);

    double vel = command_.linear.x;
    double angle_rad = command_.angular.z;


    double base = 0.05;
    double vel_r = vel + angle_rad * base / 2.0;
    double vel_l = vel - angle_rad * base / 2.0;

    set_PWM_frequency(pi, 18, std::abs(int(vel_r * 100.0)));
    set_PWM_frequency(pi, 19, std::abs(int(vel_l * 100.0)));

    if(std::abs(int(vel * 100.0)) == 0){
      motor_idle();
      return;
    }

    if(int(vel_r * 100) > 0){
      gpio_write(pi, 27, 0);
      gpio_write(pi, 22, 1);
    }else{
      gpio_write(pi, 27, 1);
      gpio_write(pi, 22, 0);
    }

    if(int(vel_l * 100) > 0){
      gpio_write(pi, 24, 0);
      gpio_write(pi, 23, 1);
    }else{
      gpio_write(pi, 24, 1);
      gpio_write(pi, 23, 0);
    }
  }

  void command_callback(const geometry_msgs::Twist& command){
    std::cerr << command << std::endl;
    command_ = command;
  }

  void init_pigpio(){
    pi = pigpio_start(NULL, NULL);
  }

  void motor_idle(){
    gpio_write(pi, 27, 0);
    gpio_write(pi, 22, 0);

    gpio_write(pi, 23, 0);
    gpio_write(pi, 24, 0);
  }

  void setup_gpio_pins(){
    set_mode(pi, 18, PI_OUTPUT);
    set_mode(pi, 19, PI_OUTPUT);

    set_PWM_range(pi, 18, 100);
    set_PWM_range(pi, 19, 100);

    set_PWM_dutycycle(pi, 18, 60);
    set_PWM_dutycycle(pi, 19, 60);

    set_mode(pi, 22, PI_OUTPUT);
    set_mode(pi, 27, PI_OUTPUT);

    set_mode(pi, 23, PI_OUTPUT);
    set_mode(pi, 24, PI_OUTPUT);
  }
};
