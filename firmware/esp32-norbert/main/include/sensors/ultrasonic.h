#ifndef ULTRASONIC_H
#define ULTRASONIC_H

void isr_ultrasonic_echo(void *arg);
void setup_ultrasonic();
void trig_ultrasonic_scan();
bool read_ultrasonic_distance(float* dist);

#endif // ULTRASONIC_H