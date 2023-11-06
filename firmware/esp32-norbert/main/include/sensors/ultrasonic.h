#ifndef ULTRASONIC_H
#define ULTRASONIC_H

#define ULTRASONIC_LOG_TAG_NAME "sensor-ultrasonic"

#define ULTRASONIC_TRIG_DURATION_US 10
#define ULTRASONIC_DIST_TIMER_MAX 100000
#define ULTRASONIC_GROUND_LEVEL_SOUND_VEL 336.1

#define ULTRASONIC_QUEUE_SIZE 10
#define ULTRASONIC_ECHO_TIMEOUT_MS 50\

void isr_ultrasonic_echo(void *arg);
void setup_ultrasonic();
void trig_ultrasonic_scan();
float read_ultrasonic_distance();

#endif // ULTRASONIC_H