const int n=4;
int dir[n];
double Kp;
double Kd;
double Ki;
int T;
int min_pwm;
int max_pwm;
double setpoint[n];
double total_error[n];
double last_error[n];
unsigned long last_time[n];


double PID(double encoder_value, int motor_no)
{
  unsigned long current_time = millis();
  unsigned long delta_time = current_time - last_time[motor_no];
  double pwm_signal;
  
  if(delta_time >= T)
  {
    double error;
    if(encoder_value < setpoint[motor_no])
    {
      error = setpoint[motor_no] - encoder_value;
      digitalWrite(dir[motor_no],HIGH);
    }
    else
    {
      error = encoder_value - setpoint[motor_no];
      digitalWrite(dir[motor_no],LOW);
    }
    total_error[motor_no] += error;
    if(total_error[motor_no] >= max_pwm)
      total_error[motor_no] = max_pwm;
    if(total_error[motor_no] <= min_pwm)
      total_error[motor_no] = min_pwm;

    double delta_error = error - last_error[motor_no];

    pwm_signal = Kp*error + (Ki*T)*total_error[motor_no] + (Kd/T)*delta_error;

    if(pwm_signal >= max_pwm)
      pwm_signal = max_pwm;
    if(pwm_signal <= min_pwm)
      pwm_signal = min_pwm;
    last_error[motor_no] = error;
    last_time[motor_no] = current_time;
  }
  return pwm_signal;
}
