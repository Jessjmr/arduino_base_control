

#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <ControlVelocidad.h>
#include <geometry_msgs/Twist.h>

float u1,u2,u3,u4, ref, Hz, v, w,wl,wr;
unsigned long timeold = 0;

ros::NodeHandle nh;

//Publisher ROS-referencia
std_msgs::Int32 info_sub1;
ros::Publisher vref("vref", &info_sub1);

//Publisher ROS-rpm
std_msgs::Float32 info_rpm;
ros::Publisher velocity("velocity", &info_rpm);

//Publisher ROS-ref smooth
std_msgs::Int32 info_refsmooth;
ros::Publisher refsmooth("refsmooth", &info_refsmooth);

void valor_ref(const std_msgs::Int32& v_ref) {
  ref = v_ref.data;
  info_sub1.data = ref;
}

void cmd_vel_cb(const geometry_msgs::Twist& cmd_msg) {
  v = cmd_msg.linear.x;
  w = cmd_msg.angular.z;
  float D, L;
  D = 0.067;
  L = 0.186;
  
  wl = (9.54929658551)*(1/D)*(2*v-w*L);
  wr = (9.54929658551)*(1/D)*(2*v+w*L);
}

//Subscriber ROS
ros::Subscriber<std_msgs::Int32> sub1("referencia_ros", valor_ref);

ros::Subscriber<geometry_msgs::Twist> cmd_vel_sub("cmd_vel", cmd_vel_cb);


//ControlVelocidad m1(1, 18, 47);
ControlVelocidad m1(1, 18, 47);
ControlVelocidad m2(2, 19, 49);
ControlVelocidad m3(3, 20, 51);
ControlVelocidad m4(4, 21, 53);

//m1.set_driver_port(1);
//m1.set_encoder_pins(18, 47);

void setup() {
  float smoother_slope, k_p, k_i,ticks_per_rev;
  
  nh.initNode();

  while (!nh.connected()) {
    nh.spinOnce();
    //nh.getHardware()->delay(10);
  }

  nh.advertise(velocity);
  nh.advertise(refsmooth);
  nh.advertise(vref);
  nh.subscribe(sub1);
  nh.subscribe(cmd_vel_sub);

  if (!nh.getParam("~smoother_slope", &smoother_slope, 1)) {
    //default values
    smoother_slope = 100;
  }

  if (!nh.getParam("~ki", &k_i, 1)) {
    //default values
    k_i = 5;
  }

  if (!nh.getParam("~kp", &k_p, 1)) {
    //default values
    k_p = 1;
  }
  
  if (!nh.getParam("~ticks_per_rev", &ticks_per_rev, 1)) {
    //default values
    ticks_per_rev = 2220;
  }
 
  //m1 = ControlVelocidad(1, 18, 47);
  
  /*
  //m1 = new ControlVelocidad(1, 18, 47);

  m1->set_smooth_slope(smoother_slope);
  m1->set_control_gains(k_p, k_i);
  m1->set_ticks_per_rev(ticks_per_rev);*/
  m1.set_smooth_slope(smoother_slope);
  m1.set_control_gains(k_p, k_i);
  m1.set_ticks_per_rev(ticks_per_rev);
  m1.set_inverted();
  
  m2.set_smooth_slope(smoother_slope);
  m2.set_control_gains(k_p, k_i);
  m2.set_ticks_per_rev(ticks_per_rev);
  m2.set_inverted();
    
  m3.set_smooth_slope(smoother_slope);
  m3.set_control_gains(k_p, k_i);
  m3.set_ticks_per_rev(ticks_per_rev);
  
  m4.set_smooth_slope(smoother_slope);
  m4.set_control_gains(k_p, k_i);
  m4.set_ticks_per_rev(ticks_per_rev);
  

  //Referencia, configurable por ROS
  ref = 0;
  Hz = 20;
}


void loop() {
  if (millis() - timeold >= (1000 / Hz)) {
    float rpm, ref_smooth;
    timeold = millis();
    

    //Motores Izquieda
    u1 = m1.update_control(wl);
    u2 = m2.update_control(wl);
    
    //Motores Derecha
    u3 = m3.update_control(wr);
    u4 = m4.update_control(wr);
    
    //rpm = m1->get_rpm();
    //ref_smooth = m1->get_ref_smooth();
    
    rpm = m2.get_rpm();
    ref_smooth = m2.get_ref_smooth();

    //Publicacion de variables en nodos de ROS
    info_rpm.data = rpm;
    info_refsmooth.data = ref_smooth;
    vref.publish(&info_sub1);
    velocity.publish(&info_rpm);
    refsmooth.publish(&info_refsmooth);
    nh.spinOnce();
  }
}
