#include "observer.hpp"

Observer::Observer() : Node("Observer")
{
  publisher = this->create_publisher<message_info::msg::Observer>("observer", 10);  // grsimへのpublisher
  timer_ = create_wall_timer(16ms, std::bind(&Observer::main, this));  // 16ms周期でtimer_callbackが実行される
  subscriber = this->create_subscription<message_info::msg::VisionDetections>(
      "vision_detections", rclcpp::QoS(10),
      std::bind(&Observer::callback_detect, this, std::placeholders::_1));  // game.cpp information subscriber
  subscriber_geometry = this->create_subscription<message_info::msg::VisionGeometry>(
      "vision_geometry", rclcpp::QoS(10),
      std::bind(&Observer::callback_geometry, this, std::placeholders::_1));  // game.cpp information subscriber
  our_robots_quanity = ROBOT_QUANITY;
}

void Observer::main()
{
  Observer::ball_status();
  Observer::robot_status();
  Observer::enemy_status();
  Observer::invasion_status();
}

void Observer::invasion_status()
{
  bool invasion_flag[16];
  for(int i = 0; i < 16; i++)
    invasion_flag[i] = false;

    float field_length ;
    float field_width ;

   geometry_msgs::msg::Pose2D p2_pose;

   for(auto line : geometry.field_lines)
   {
     if(line.name == "RightPenaltyStretch")
     {
       field_length = line.p2_x;
       field_width = line.p2_y;
     }
   }

    //cout<< field_length  <<endl;

  for(auto robot : frame.our_robots)
  {
    if(robot.robot_id != 0)
    {
      if( fabs(robot.pose.y) >= 4.5 || fabs(robot.pose.x) >= 6 )
      {
        invasion_flag[robot.robot_id] = true;
        cout << robot.robot_id << endl;
      }
      else if( fabs(robot.pose.x) >= field_length && fabs(robot.pose.y) <= field_width )
      {
        invasion_flag[robot.robot_id] = true;
        cout << robot.robot_id << endl;
      }
    }
  }
}


void Observer::ball_status()
{
  float ball_difference_x = pre_ball.x - ball.pose.x;
  float ball_difference_y = pre_ball.y - ball.pose.y;

  if (ball_difference_y != 0 && ball_difference_x != 0)
  {
    ball_slope = ball_difference_y / ball_difference_x;                                //傾き
    ball_intercept = ball.pose.y - (ball_slope * ball.pose.x);                         //切片
    ball_speed = sqrt(pow(ball_difference_x, 2) + pow(ball_difference_y, 2)) / 0.016;  //ボールの速さ
  }
  else
  {
    ball_slope = 0;
    ball_intercept = ball.pose.y;
    ball_speed = 0;
  }

  pre_ball.x = ball.pose.x;  //前回のボールの座標を記憶
  pre_ball.y = ball.pose.y;

  send_status.ball.slope = ball_slope;
  send_status.ball.intercept = ball_intercept;
  send_status.ball.speed = ball_speed;
  // cout<<ball_speed<<endl;

  this->publisher->publish(send_status);
}

void Observer::robot_status()
{
  for (auto robot : frame.our_robots)
  {
    int i = robot.robot_id;
    robot_difference_x[i] = pre_robot[i].x - robot.pose.x;
    robot_difference_y[i] = pre_robot[i].y - robot.pose.y;
    robot_difference_theta[i] = pre_robot[i].theta - robot.pose.theta;

    if (robot_difference_y[i] != 0 && robot_difference_x[i] != 0)
    {
      robot_slope = robot_difference_y[i] / robot_difference_x[i];                                //傾き
      robot_intercept = robot.pose.y - (robot_slope * robot.pose.x);                              //切片
      robot_speed = sqrt(pow(robot_difference_x[i], 2) + pow(robot_difference_y[i], 2)) / 0.016;  //速さ
    }
    else
    {
      robot_slope = 0;
      robot_intercept = our_robots[i].pose.y;
      robot_speed = 0;
    }
    robot_angular_velocity = robot_difference_theta[i] / 0.016;  //角速度

    pre_robot[i].x = robot.pose.x;  //前回のボールの座標を記憶
    pre_robot[i].y = robot.pose.y;

    send_status.robot[i].slope = robot_slope;
    send_status.robot[i].intercept = robot_intercept;
    send_status.robot[i].speed = robot_speed;
    send_status.robot[i].angular_velocity = robot_angular_velocity;
  }
  // cout<<frame.our_robots[0].pose.x<<endl;

  this->publisher->publish(send_status);
}

void Observer::enemy_status()
{
  for (auto enemy : frame.their_robots)
  {
    int i = enemy.robot_id;
    enemy_difference_x[i] = pre_enemy[i].x - enemy.pose.x;
    enemy_difference_y[i] = pre_enemy[i].y - enemy.pose.y;
    enemy_difference_theta[i] = pre_enemy[i].theta - enemy.pose.theta;

    if (enemy_difference_y[i] != 0 && enemy_difference_x[i] != 0)
    {
      enemy_slope = enemy_difference_y[i] / enemy_difference_x[i];                                //傾き
      enemy_intercept = enemy.pose.y - (enemy_slope * enemy.pose.x);                              //切片
      enemy_speed = sqrt(pow(enemy_difference_x[i], 2) + pow(enemy_difference_y[i], 2)) / 0.016;  //速さ
    }
    else
    {
      enemy_slope = 0;
      enemy_intercept = our_robots[i].pose.y;
      enemy_speed = 0;
    }
    enemy_angular_velocity = enemy_difference_theta[i] / 0.016;  //角速度

    pre_enemy[i].x = enemy.pose.x;  //前回のボールの座標を記憶
    pre_enemy[i].y = enemy.pose.y;

    send_status.enemy[i].slope = enemy_slope;
    send_status.enemy[i].intercept = enemy_intercept;
    send_status.enemy[i].speed = enemy_speed;
    send_status.enemy[i].angular_velocity = enemy_angular_velocity;
  }

  this->publisher->publish(send_status);
}

void Observer::callback_detect(const message_info::msg::VisionDetections::SharedPtr vision_message)
{  // subscriber起動
  frame.our_robots = vision_message->frame.our_robots;
  frame.their_robots = vision_message->frame.their_robots;
  ball = vision_message->frame.ball;
  our_quanity = vision_message->frame.our_quanity;

  color_value = 1;
  if (vision_message->my_color == "yellow")  //色の値受信
    color_value = -1;
}

void Observer::callback_geometry(const message_info::msg::VisionGeometry::SharedPtr geometry_message)
{
  geometry = geometry_message->field;
}
