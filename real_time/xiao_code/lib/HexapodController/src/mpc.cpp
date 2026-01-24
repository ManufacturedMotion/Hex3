// #include "joint_controller_dyn.h"

// /* Utility clamp */
// static float MPCController::clamp(float v, float lo, float hi)
// {
//     if (v < lo) return lo;
//     if (v > hi) return hi;
//     return v;
// }

// void MPCController::joint_controller_dyn_init(JointControllerDyn* ctrl,
//                                float kp_pos,
//                                float kv_vel,
//                                float ki_vel,
//                                float kff_vel,
//                                float kacc,
//                                float duty_limit,
//                                float integral_limit)
// {
//     kp_pos = kp_pos;
//     kv_vel = kv_vel;
//     ki_vel = ki_vel;
//     kff_vel = kff_vel;
//     kacc = kacc;

//     duty_limit =
//         (duty_limit > 0.0f) ? duty_limit : -duty_limit;

//     integral_limit =
//         (integral_limit > 0.0f) ? integral_limit : -integral_limit;

//     vel_integral = 0.0f;
// }

// float MPCController::joint_controller_dyn_update(,
//                                   float q,
//                                   float qdot,
//                                   float q_ref,
//                                   float qdot_ref,
//                                   float qddot_ref,
//                                   float u_grav,
//                                   float dt)
// {
//     /* Outer loop: position → desired velocity */
//     float qdot_des =
//         qdot_ref + kp_pos * (q_ref - q);

//     /* Velocity error */
//     float vel_err = qdot_des - qdot;

//     /* Candidate integrator update */
//     float new_integral =
//         vel_integral + vel_err * dt;

//     new_integral = clamp(new_integral,
//                          -integral_limit,
//                           integral_limit);

//     /* Unsaturated control output */
//     float u =
//         kv_vel * vel_err +
//         ki_vel * new_integral +
//         kff_vel * qdot_des +
//         kacc * qddot_ref +
//         u_grav;

//     /* Saturation */
//     float u_sat = clamp(u,
//                         -duty_limit,
//                          duty_limit);

//     /* Anti-windup: accept integrator only if not saturated */
//     if (u == u_sat) {
//         vel_integral = new_integral;
//     }

//     return u_sat;
// }
