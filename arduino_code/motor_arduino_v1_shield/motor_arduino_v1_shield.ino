/*
 * rosserial PubSub Example
 * Prints "hello world!" and toggles led
 */
#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
//Libreria Encoder
#include <Encoder.h>
//Libreria Motor
#include <AFMotor.h>

float t1=0,u,t=0,int_e=0,kp,ki,ref,int_bound,ref_ros;
float a,ref_smooth=0,Hz;
long g_ant,grados;
unsigned long timeold = 0;

ros::NodeHandle  nh;

//Publisher ROS-referencia
std_msgs::Int32 info_sub1;
ros::Publisher vref("vref", &info_sub1);

//Publisher ROS-rpm
std_msgs::Float32 info_rpm;
ros::Publisher velocity("velocity", &info_rpm);

//Publisher ROS-ref smooth
std_msgs::Int32 info_refsmooth;
ros::Publisher refsmooth("refsmooth", &info_refsmooth);

void valor_ref( const std_msgs::Int32& v_ref){
  ref=v_ref.data;
  info_sub1.data=ref;
}

//Subscriber ROS
ros::Subscriber<std_msgs::Int32> sub1("referencia_ros", valor_ref);


// Motor
AF_DCMotor motor(2);
  // Encoder
Encoder myEnc(18, 26);
  
void setup()
{
  Serial.println("entering setup");
  // Medición de tiempo
  t1=float(millis())/1000; 
  // Cambio de pasos de encoder a grados= 2220 pulsos por revolución entre 360 grados
  g_ant=grados_encoder();

  nh.initNode();
  nh.advertise(velocity);
  nh.advertise(refsmooth);
  nh.advertise(vref);
  nh.subscribe(sub1);
  
  a=100;
  kp=1;
  ki=5;

  // saturación de la integral 
  int_bound=1.8;

  //Referencia, configurable por ROS
  ref=0;
  //a = 600;

  //Hertz
  Hz=20; 
}

float grados_encoder()
{
  grados=0.162162162*myEnc.read();

  return grados;
}

float mov_motor(float u)
{
  if (u>0){
  motor.run(BACKWARD);}
  else  {
  motor.run(FORWARD);}

  motor.setSpeed(abs(u));
}

float ref_suave (float ref)
{
//Función de referencia suave
  
  if (abs(ref-ref_smooth)<abs(a/Hz)){
    ref_smooth=ref;        
  }
  else if(ref_smooth>ref){
    if(abs(ref-ref_smooth)>90){
      ref_smooth = ref_smooth - 0.3*(a/Hz);      
    }
    else{         
      ref_smooth = ref_smooth - (a/Hz);
    }    
  } else if (ref_smooth<ref) {
    if(abs(ref-ref_smooth)>90){
      ref_smooth = ref_smooth + 0.3*(a/Hz);      
    }
    else{         
      ref_smooth = ref_smooth + (a/Hz);
    }
  }  
  return ref_smooth;
}

int sat (float x,float cota)
{
//Función de saturación
  
  if (x>cota){
      x=cota;
  }
  else if (x<-cota){
      x=-cota;
  }
  return x;
}

void loop()
{
if (millis() - timeold >= (1000/Hz)) {
  float t2; // Variable de tiempo en el instante actual
  float dt; //Variable diferencial de tiempo (instante actual)-(instante anterior)
  float rpm; // Variable Revoluciones por Minuto RPM=(Grados/segundo)/6
  long grados_actuales; // Variable de posición del encoder actual
  float d_finitas; //Calculo de velocidad usando diferencias finitas en grados/segundo
  float e; // Variable del error Velocidad de referencia - velocidad actual

  // Lectura de tiempo
  t2=float(millis())/1000;
  dt=t2-t1;
  
  timeold = millis(); 
  // Lectura actual de posición
  grados_actuales = grados_encoder();
  // Calculo de la velocidad usando derivada sucia (diferencias finitas)
  d_finitas=(grados_actuales-g_ant)/(dt);
  // Grados a RPM  
  rpm=d_finitas/6;
  ref_smooth=ref_suave(ref);
  e=ref_smooth-rpm; //Error
  if (abs(e)<10){
    int_e=int_e+e*dt; // Integral del error
  }
  // Saturación del integrador
  int_e= sat(int_e,int_bound); 
  //Control PI
  u=(255/120)*(rpm+(kp*e)+(ki*int_e));
  // Saturación del controlador
  u=sat(u,255); 

  mov_motor(u);
  
  vref.publish(&info_sub1);  
  info_rpm.data=rpm;
  velocity.publish(&info_rpm);
  info_refsmooth.data=ref_smooth;
  refsmooth.publish(&info_refsmooth);
  nh.spinOnce();

  t1=t2;
  g_ant=grados_actuales;
}
}
