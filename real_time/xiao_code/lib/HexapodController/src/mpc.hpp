// #ifndef JOINT_CONTROLLER_DYN_H
// #define JOINT_CONTROLLER_DYN_H

// class MPCController{
//     /* Position loop */
//     float kp_pos;

//     /* Velocity loop */
//     float kv_vel;
//     float ki_vel;
//     float kff_vel;

//     /* Acceleration (inertia) feedforward */
//     float kacc;

//     /* Output limits */
//     float duty_limit;

//     /* Integrator */
//     float vel_integral;
//     float integral_limit;

//     void joint_controller_dyn_init(float kp_pos,
//                                 float kv_vel,
//                                 float ki_vel,
//                                 float kff_vel,
//                                 float kacc,
//                                 float duty_limit,
//                                 float integral_limit);

//     /**
//      * Update controller
//      *
//      * @param q          joint position [rad]
//      * @param qdot       joint velocity [rad/s]
//      * @param q_ref      desired position [rad]
//      * @param qdot_ref   desired velocity [rad/s]
//      * @param qddot_ref  desired acceleration [rad/s^2]
//      * @param u_grav     gravity feedforward duty term
//      * @param dt         timestep [s]
//      */
//     float joint_controller_dyn_update(float q,
//                                   float qdot,
//                                   float q_ref,
//                                   float qdot_ref,
//                                   float qddot_ref,
//                                   float u_grav,
//                                   float dt);

//     static float MPCController::clamp(float v, float lo, float hi)
//     {
//         if (v < lo) return lo;
//         if (v > hi) return hi;
//         return v;
//     }

// };

// #endif /* JOINT_CONTROLLER_DYN_H */
