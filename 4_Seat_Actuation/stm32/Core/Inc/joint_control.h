#ifndef JOINT_CONTROL_H
#define JOINT_CONTROL_H
void Control_Init(void);
void Control_Poll(void);
int Control_Command(const char *command);
void Control_Heartbeat(void);
void Control_Fault(const char *reason);
void FSR_Transmit(const char *text);
int FSR_TryTelemetry(const char *text);
#endif
