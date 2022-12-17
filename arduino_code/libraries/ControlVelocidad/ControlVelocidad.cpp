/*
  ControlVelocidad.cpp - Library for PI velocity control.
  Created by Aguillon Nestor and Maldonado Jessica, Dic 10, 2022.
  Released into the public domain.
*/

#include <Arduino.h>
#include <ControlVelocidad.h>

ControlVelocidad::ControlVelocidad(int mi, int a_encoder, int b_encoder)
  : mi_(mi), myEnc_(a_encoder, b_encoder) {
  g_ant_ = 0;
  t1_ = float(millis()) / 1000;
  is_inverted_ = false;
  ref_smooth_ = 0;
  a_ = 1000;
  int_bound_ = 1.8;
  int_e_ = 0;
  kp_ = 1;
  ki_ = 5;
  a_  = 500;
}

void ControlVelocidad::set_smooth_slope(float slope) {
  a_ = slope;
}

void ControlVelocidad::set_control_gains(float kp, float ki) {
  kp_ = kp;
  ki_ = ki;
}



void ControlVelocidad::set_ticks_per_rev(float ticks_per_rev){
  ticks_per_rev_ = ticks_per_rev;
}

void ControlVelocidad::set_int_bound(float int_bound){
  int_bound_ = int_bound;
}

void ControlVelocidad::set_inverted(){
  is_inverted_ = true;	
}

void ControlVelocidad::set_driver_port(int mi){
  mi_=AF_DCMotor(mi);
}

void ControlVelocidad::set_encoder_pins(int a_encoder, int b_encoder){
 myEnc_=Encoder(a_encoder,b_encoder);
}


float ControlVelocidad::update_control(float ref) {
  float d_finitas;  //Calculo de velocidad usando diferencias finitas en grados/segundo
  float e;          // Variable del error Velocidad de referencia - velocidad actual
  float dt;         //Variable diferencial de tiempo (instante actual)-(instante anterior)
  // Lectura de tiempo
  t2_ = float(millis()) / 1000;
  dt = t2_ - t1_;
  // Cambio de pasos de encoder a grados= 2220 pulsos por revolución entre 360 grados; 0.162162162
  grados_actuales_ = (360/ticks_per_rev_) * myEnc_.read();
  // Calculo de la velocidad usando derivada sucia (diferencias finitas)
  d_finitas = (grados_actuales_ - g_ant_) / (dt);
  // Grados a RPM
  rpm_ = d_finitas / 6;
  if (is_inverted_) {
    rpm_ = -rpm_;
  }
  ref_suave(ref);
  e = ref_smooth_ - rpm_;  //Error

  if (abs(e) < 10) {
    int_e_ = int_e_ + e * dt;  // Integral del error
  }
  // Saturación del integrador
  int_e_ = sat(int_e_, int_bound_);
  //Control PI
  u_ = (255 / 120) * (rpm_ + (kp_ * e) + (ki_ * int_e_));
  // Saturación del controlador
  u_ = sat(u_, 255);

  //Actualización de variables
  t1_ = t2_;
  g_ant_ = grados_actuales_;

  if (is_inverted_) {
    u_ = -u_;
  }

  mov_motor();

  return u_;
}

void ControlVelocidad::mov_motor() {
  if (u_ < 0) {
    mi_.run(BACKWARD);
  } 
  else {
    mi_.run(FORWARD);
  }

  mi_.setSpeed(abs(u_));
}

float ControlVelocidad::get_rpm() {
  return rpm_;
}

float ControlVelocidad::get_ref_smooth() {
  return ref_smooth_;
}

void ControlVelocidad::ref_suave(float ref) {
  //Función de referencia suave

  if (abs(ref - ref_smooth_) < abs(a_ * (t2_ - t1_))) {
    ref_smooth_ = ref;
  } else if (ref_smooth_ > ref) {
    if (abs(ref - ref_smooth_) > 90) {
      ref_smooth_ = ref_smooth_ - 0.3 * (a_ * (t2_ - t1_));
    } else {
      ref_smooth_ = ref_smooth_ - (a_ * (t2_ - t1_));
    }
  } else if (ref_smooth_ < ref) {
    if (abs(ref - ref_smooth_) > 90) {
      ref_smooth_ = ref_smooth_ + 0.3 * (a_ * (t2_ - t1_));
    } else {
      ref_smooth_ = ref_smooth_ + (a_ * (t2_ - t1_));
    }
  }
  //return ref_smooth;
}

float ControlVelocidad::sat(float x, float cota) {
  //Función de saturación

  if (x > cota) {
    x = cota;
  } else if (x < -cota) {
    x = -cota;
  }
  return x;
}
