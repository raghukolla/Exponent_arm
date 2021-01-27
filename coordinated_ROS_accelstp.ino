// Include the AccelStepper library:
#include <AccelStepper.h>
#include <Servo.h>

// Rosserial Arduino Library - Version: 0.7.9
#include <ros.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32MultiArray.h>

ros::NodeHandle  nh;

std_msgs::Int32 temp_msg;
ros::Publisher pub_temp("temperature", &temp_msg);

std_msgs::Float32MultiArray msg_arr;
ros::Publisher Pub_Arr("array_test", &msg_arr);


// Define stepper motor connections and motor interface type. Motor interface type must be set to 1 when using a driver:
#define dirPin_base 15
#define stepPin_base 16

#define dirPin_shld 9
#define stepPin_shld 8

#define dirPin_elbw 10
#define stepPin_elbw 11

#define dirPin_wrst1 21
#define stepPin_wrst1 22
#define enblPin_wrst1 20

#define dirPin_wrst2 19
#define stepPin_wrst2 18
#define enblPin_wrst2 17

#define motorInterfaceType 1

// velocity and acceleration limits of joints
// order of joints -  base, shld, elbw, wrist1, wrist2
int max_velocities[5] = {2000, 3000, 4000, 3500, 3000};
int max_accelerations[5] = {2000, 3000, 3000, 4000, 4000};

// Create new instances of the AccelStepper class:
AccelStepper stepper_base = AccelStepper(motorInterfaceType, stepPin_base, dirPin_base);
AccelStepper stepper_shld = AccelStepper(motorInterfaceType, stepPin_shld, dirPin_shld);
AccelStepper stepper_elbw = AccelStepper(motorInterfaceType, stepPin_elbw, dirPin_elbw);
AccelStepper stepper_wrst1 = AccelStepper(motorInterfaceType, stepPin_wrst1, dirPin_wrst1);
AccelStepper stepper_wrst2 = AccelStepper(motorInterfaceType, stepPin_wrst2, dirPin_wrst2);

Servo servo_wrist3;
Servo gripper;

int wrist3_pos = 0;
int wrist3_target = 0; 
unsigned long prev_millis = millis();
unsigned long curr_millis;
int servo_speed = 30; // degrees per second
int servo_interval = 1000/servo_speed;

float time_to_move(int accel, int max_vel, int dist)
{
  if(dist==0) return 0.0;
  dist = abs(dist);
  float accel_dist = (max_vel * max_vel)/float(2*accel);
  if(accel_dist > (dist/2)) accel_dist = (dist/2.0);
  else accel_dist = accel_dist;

  float vel_limit = sqrt(float(2.0)*accel*accel_dist);
  float accel_time = vel_limit/accel;
  float vel_time = (dist - 2*accel_dist)/vel_limit;
  float total_time = 2*accel_time + vel_time; // seconds

  return total_time;
}

void set_vel_acl(int max_vels[], int acls[])
{
  // Set the maximum speed and acceleration:
  stepper_base.setMaxSpeed(max_vels[0]);
  stepper_base.setAcceleration(acls[0]);
  
  stepper_shld.setMaxSpeed(max_vels[1]);
  stepper_shld.setAcceleration(acls[1]);
  
  stepper_elbw.setMaxSpeed(max_vels[2]);
  stepper_elbw.setAcceleration(acls[2]);
  
  stepper_wrst1.setMaxSpeed(max_vels[3]);
  stepper_wrst1.setAcceleration(acls[3]);

  stepper_wrst2.setMaxSpeed(max_vels[4]);
  stepper_wrst2.setAcceleration(acls[4]);
}

void open_gripper(){
   for(int pos = 90; pos>=35; pos-=1)     // goes from 180 degrees to 0 degrees 
    {                                
      gripper.write(pos);              // tell servo to go to position in variable 'pos' 
      delay(15);                       // waits 15ms for the servo to reach the position 
    } 
}

void close_gripper(){
  for(int pos = 35; pos <= 90; pos += 1) // goes from 0 degrees to 180 degrees 
  {                                  // in steps of 1 degree 
    gripper.write(pos);              // tell servo to go to position in variable 'pos' 
    delay(15);                       // waits 15ms for the servo to reach the position 
  }
}

void moveAll_ToPosition(const std_msgs::Int32MultiArray& positions){
  // Define required arrays
  int dist_to_go[5]; 
  float move_times[5];
  float time_ratios[5];
  int new_max_vels[5];
  int accelerations[5];

  dist_to_go[0] = positions.data[1] - stepper_base.currentPosition();
  dist_to_go[1] = positions.data[2] - stepper_shld.currentPosition();
  dist_to_go[2] = positions.data[0] - stepper_elbw.currentPosition();
  dist_to_go[3] = positions.data[3] - stepper_wrst1.currentPosition();
  dist_to_go[4] = positions.data[4] - stepper_wrst2.currentPosition();
  
  for(int i=0; i<5; i++)
  {
    move_times[i] = time_to_move(max_accelerations[i], max_velocities[i], dist_to_go[i]);
  }
  float max_time = move_times[0];
  
  for(int i=0; i<5; i++)
  {
    max_time = max(max_time, move_times[i]);
  }

  for(int i=0; i<5; i++)
  {
    time_ratios[i] = move_times[i]/max_time;
    new_max_vels[i] = max_velocities[i]*(time_ratios[i]);
    accelerations[i] = max_accelerations[i]*(time_ratios[i]*time_ratios[i]);
  }

  msg_arr.data = time_ratios;
  msg_arr.data_length =5;
  Pub_Arr.publish(&msg_arr);
  set_vel_acl(new_max_vels, accelerations);

  float dist_to_goflt[5] = {float(dist_to_go[0]),float(dist_to_go[1]),float(dist_to_go[2]),float(dist_to_go[3]),float(dist_to_go[4])};
  msg_arr.data = dist_to_goflt;
  msg_arr.data_length =5;
  Pub_Arr.publish(&msg_arr);
  set_vel_acl(new_max_vels, accelerations);

  msg_arr.data = move_times;
  msg_arr.data_length =5;
  Pub_Arr.publish(&msg_arr);
  set_vel_acl(new_max_vels, accelerations);

  float time_max[1] = {max_time};
  msg_arr.data = time_max;
  msg_arr.data_length =1;
  Pub_Arr.publish(&msg_arr);
  set_vel_acl(new_max_vels, accelerations);
  
  // Set the target position:
  stepper_base.moveTo(positions.data[1]);
  stepper_shld.moveTo(positions.data[2]);
  stepper_elbw.moveTo(positions.data[0]);
  stepper_wrst1.moveTo(positions.data[3]);
  stepper_wrst2.moveTo(positions.data[4]);
  wrist3_target = positions.data[5];

  while((stepper_shld.distanceToGo() != 0) || (stepper_base.distanceToGo() != 0) || (stepper_elbw.distanceToGo() != 0)
        || (stepper_wrst1.distanceToGo() != 0) || (stepper_wrst2.distanceToGo() != 0) || (wrist3_pos != wrist3_target) )
  {
    stepper_base.run();
    stepper_shld.run();
    stepper_elbw.run();
    stepper_wrst1.run();
    stepper_wrst2.run();

    curr_millis = millis();
    if( (curr_millis - prev_millis) >= servo_interval ){
      prev_millis = curr_millis;
      if(wrist3_target > wrist3_pos){
        wrist3_pos = wrist3_pos+1;
        servo_wrist3.write(wrist3_pos);
      }
      if(wrist3_target < wrist3_pos){
        wrist3_pos = wrist3_pos-1;
        servo_wrist3.write(wrist3_pos);
      }
    }
  }
  close_gripper();
  open_gripper();
}

void messageCb(const std_msgs::Int32MultiArray& joint_steps){
  for(int i=0; i <6; i++){
  temp_msg.data = joint_steps.data[i];
  pub_temp.publish(&temp_msg);
  }
  moveAll_ToPosition(joint_steps);
}


ros::Subscriber<std_msgs::Int32MultiArray> sub("ard_pub", messageCb);

void setup() {

  nh.initNode();
  nh.subscribe(sub);  
  nh.advertise(pub_temp);
  nh.advertise(Pub_Arr);
  
  // Set the maximum speed and acceleration:
  stepper_base.setMaxSpeed(2000);
  stepper_base.setAcceleration(2000);
  
  stepper_shld.setMaxSpeed(4000);
  stepper_shld.setAcceleration(4000);
  
  stepper_elbw.setMaxSpeed(4000);
  stepper_elbw.setAcceleration(3000);
  
  stepper_wrst1.setMaxSpeed(3500);
  stepper_wrst1.setAcceleration(4000);

  stepper_wrst2.setMaxSpeed(3000);
  stepper_wrst2.setAcceleration(4000);
  
  pinMode(enblPin_wrst1 , OUTPUT);
  digitalWrite(enblPin_wrst1, LOW);
  
  pinMode(enblPin_wrst2 , OUTPUT);
  digitalWrite(enblPin_wrst2, LOW);

  servo_wrist3.attach(6);
  gripper.attach(7);
  
  delay(2000);
  open_gripper(); 
}



void loop() {
  
  nh.spinOnce();
  delay(1);
}
