/*
  ControlVelocidad.h - Library for PI velocity control.
  Created by Aguillon Nestor and Maldonado Jessica, Dic 10, 2022.
  Released into the public domain.
*/
#ifndef ControlVelocidad_h
#define ControlVelocidad_h

#include <Arduino.h>
//Libreria Encoder
#include <Encoder.h>
//Libreria Motor
#include <AFMotor.h>

class ControlVelocidad {
public:
  ControlVelocidad(int mi, int a_encoder, int b_encoder);
  void set_driver_port(int mi);
  void set_encoder_pins(int a_encoder, int b_encoder);
  float update_control(float ref);
  void set_ticks_per_rev(float ticks_per_rev);
  void set_int_bound(float int_bound);
  void set_inverted();
  float get_rpm();
  float get_ref_smooth();
  void set_smooth_slope(float slope);
  void set_control_gains(float kp, float ki);
private:
  AF_DCMotor mi_;
  Encoder myEnc_;
  float ref_smooth_;
  void ref_suave(float ref);
  float sat(float x, float cota);
  void mov_motor();
  float g_ant_;
  float t1_;
  float t2_;
  bool is_inverted_;
  float a_;
  float int_bound_;
  float int_e_;
  float rpm_;             // Variable Revoluciones por Minuto RPM=(Grados/segundo)/6
  long grados_actuales_;  // Variable de posición del encoder actual
  float u_;               // Variable de control
  float kp_;              // Ganancia control propocional
  float ki_;              // Ganancia control integral
  float ticks_per_rev_;   // Pulsos por revolucion
};

#endif
