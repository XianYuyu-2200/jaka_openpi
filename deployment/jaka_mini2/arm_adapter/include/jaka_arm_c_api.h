#pragma once

#ifdef __cplusplus
extern "C" {
#endif

void* jaka_arm_create(const char* robot_ip, int allow_motion);
void jaka_arm_destroy(void* handle);
int jaka_arm_connect(void* handle);
int jaka_arm_disconnect(void* handle);
int jaka_arm_read_state(void* handle, double joint_position_rad[6]);
int jaka_arm_forward_kinematics(void* handle, const double joint_position_rad[6], double tcp_pose[6]);
int jaka_arm_arm_motion(void* handle);
int jaka_arm_disarm_motion(void* handle);
int jaka_arm_send_servo_j(void* handle, const double joint_position_rad[6]);
int jaka_arm_stop(void* handle);
int jaka_o6_read_state(void* handle, double hand_position[6]);
int jaka_o6_send_position_direct(void* handle, const double hand_position[6]);

#ifdef __cplusplus
}
#endif
