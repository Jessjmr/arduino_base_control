

#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <ControlVelocidad.h>

float u1,u2,u3,u4, ref, Hz;
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

//Subscriber ROS
ros::Subscriber<std_msgs::Int32> sub1("referencia_ros", valor_ref);


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
  
  
  m2.set_smooth_slope(smoother_slope);
  m2.set_control_gains(k_p, k_i);
  m2.set_ticks_per_rev(ticks_per_rev);
    
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

    //Actualizar y obtener la nueva variable de control
    //u1 = m1->update_control(ref);
    u1 = m1.update_control(ref);
    u2 = m2.update_control(ref);
    u3 = m3.update_control(ref);
    u4 = m4.update_control(ref);
    
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
