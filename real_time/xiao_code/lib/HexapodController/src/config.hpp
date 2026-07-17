#include "axis.hpp"
#include <stdbool.h>
#include "user_config.hpp"

#ifndef HEXA_CONFIG
#define HEXA_CONFIG

   #define CALIBRATION_OFFSET_A0 1.047198
   #define CALIBRATION_OFFSET_A1 1.56976
   #define CALIBRATION_OFFSET_A2 -1.841322

   #ifdef ZACK
      #define STEP_THRESHOLD 40 
      #define FIFO_IDLE_THRESHOLD 100
      #define REFERENCE_POINTS
      #define ZERO_POINTS { \
         {\
            -2.662991 + CALIBRATION_OFFSET_A0,  \
            -0.852893 + CALIBRATION_OFFSET_A1,  \
            -0.836020 + CALIBRATION_OFFSET_A2}, \
         {\
            -2.218136 + CALIBRATION_OFFSET_A0, \
            -2.822525 + CALIBRATION_OFFSET_A1, \
            -0.051068  + CALIBRATION_OFFSET_A2}, \
         {\
            -2.435961 + CALIBRATION_OFFSET_A0, \
            0.365087  + CALIBRATION_OFFSET_A1,  \
            -2.998932 + CALIBRATION_OFFSET_A2},\
         {\
            0.920388  + CALIBRATION_OFFSET_A0,  \
            0.897379  + CALIBRATION_OFFSET_A1,  \
            1.992971  + CALIBRATION_OFFSET_A2}, \
         {\
            1.911340  + CALIBRATION_OFFSET_A0,  \
            0.579845  + CALIBRATION_OFFSET_A1,  \
            0.069029  + CALIBRATION_OFFSET_A2}, \
         {\
            0.822214  + CALIBRATION_OFFSET_A0,  \
            -2.100020 + CALIBRATION_OFFSET_A1,  \
            3.072563  + CALIBRATION_OFFSET_A2}  \
    }
    
    #define SCALE_FACT {{1, 1, 1}, {1, 1, 1}, {1, 1, 1}, {1, 1, 1}, {1, 1, 1}, {1, 1, 1}}
    #define TOE_IDLE_READ (float[]){0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f} 
    #define REVERSE_AXIS {{false, true, true}, {false, true, true}, {false, true, true}, {false, true, true}, {false, true, true}, {false, true, true}}
#endif

   #ifdef DILLON
      #define STEP_THRESHOLD 40 
      #define FIFO_IDLE_THRESHOLD 100
      #define ZERO_POINTS {                        \
         {                                         \
            1.193437 + CALIBRATION_OFFSET_A0,      \
            -1.839243 + CALIBRATION_OFFSET_A1,     \
            2.647651 + CALIBRATION_OFFSET_A2},     \
         {                                         \
            0.070563 + CALIBRATION_OFFSET_A0,      \
            0.398835 + CALIBRATION_OFFSET_A1,     \
            -2.417554 + CALIBRATION_OFFSET_A2},    \                                     
         { \
            1.849981 + CALIBRATION_OFFSET_A0,      \
            -0.187146 + CALIBRATION_OFFSET_A1,     \
            -0.581379 + CALIBRATION_OFFSET_A2},    \   
         { \ 
            0.447922 + CALIBRATION_OFFSET_A0,      \
            1.087592 + CALIBRATION_OFFSET_A1,     \
            1.388253 + CALIBRATION_OFFSET_A2},    \   
         { \
            0.401903 + CALIBRATION_OFFSET_A0,      \
            -2.394544 + CALIBRATION_OFFSET_A1,     \
            0.748583 + CALIBRATION_OFFSET_A2},    \    
         { \
            -1.770214 + CALIBRATION_OFFSET_A0,      \
            -2.675262 + CALIBRATION_OFFSET_A1,     \
            -0.562971 + CALIBRATION_OFFSET_A2},    \   
    }
    #define SCALE_FACT {{1, 1, 1}, {1, 1, 1}, {1, 1, 1}, {1, 1, 1}, {1, 1, 1}, {1, 1, 1}}
    #define TOE_IDLE_READ (float[]){26.7f, 19.0f, 25.5f, 29.3f, 28.32f, 26.4f} 
    #define REVERSE_AXIS {{true, false, false}, {true, false, false}, {true, false, false}, {true, false, false}, {true, false, false}, {true, false, false}}
#endif

#ifdef DANNY
   #define STEP_THRESHOLD 40
   #define FIFO_IDLE_THRESHOLD 100
   #define ZERO_POINTS {                    \
      {0.547631 + CALIBRATION_OFFSET_A0,   \
      -0.836020 + CALIBRATION_OFFSET_A1,  \
      2.787243 + CALIBRATION_OFFSET_A2},  \
      {2.426758 + CALIBRATION_OFFSET_A0,   \
      -0.886641 + CALIBRATION_OFFSET_A1,  \
      -1.106000 + CALIBRATION_OFFSET_A2}, \
      {1.334563 + CALIBRATION_OFFSET_A0,   \
      -1.788622 + CALIBRATION_OFFSET_A1,  \
      2.397612 + CALIBRATION_OFFSET_A2}, \
      {-2.078544 + CALIBRATION_OFFSET_A0,   \
      2.235010 + CALIBRATION_OFFSET_A1,  \
      2.583224 + CALIBRATION_OFFSET_A2}, \
      {0.305262 + CALIBRATION_OFFSET_A0,   \
      -0.075165 + CALIBRATION_OFFSET_A1,  \
      1.826971 + CALIBRATION_OFFSET_A2}, \
      {1.481825 + CALIBRATION_OFFSET_A0,   \
      -2.913029 + CALIBRATION_OFFSET_A1,  \
      -3.129321 + CALIBRATION_OFFSET_A2}, \
   }
   #define SCALE_FACT {{1, 1, 1}, {1, 1, 1}, {1, 1, 1}, {1, 1, 1}, {1, 1, 1}, {1, 1, 1}}
   #define TOE_IDLE_READ (float[]){0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f} 
   #define REVERSE_AXIS                                                                                                                       \
      {                                                                                                                                       \
         {true, false, false}, {true, false, false}, {true, false, false}, {true, false, false}, {true, false, false}, { true, false, false } \
      }
#endif

#define MIN_POS {{-CALIBRATION_OFFSET_A0, -CALIBRATION_OFFSET_A1 + 0.05, CALIBRATION_OFFSET_A2 + 0.05}, \
                 {-CALIBRATION_OFFSET_A0, -CALIBRATION_OFFSET_A1 + 0.05, CALIBRATION_OFFSET_A2 + 0.05},   \
                 {-CALIBRATION_OFFSET_A0, -CALIBRATION_OFFSET_A1 + 0.05, CALIBRATION_OFFSET_A2 + 0.05},   \
                 {-CALIBRATION_OFFSET_A0, -CALIBRATION_OFFSET_A1 + 0.05, CALIBRATION_OFFSET_A2 + 0.05},   \
                 {-CALIBRATION_OFFSET_A0, -CALIBRATION_OFFSET_A1 + 0.05, CALIBRATION_OFFSET_A2 + 0.05},   \
                 {-CALIBRATION_OFFSET_A0, -CALIBRATION_OFFSET_A1 + 0.05, CALIBRATION_OFFSET_A2 + 0.05}}
#define MAX_POS {{CALIBRATION_OFFSET_A0, CALIBRATION_OFFSET_A1 - 0.05, -CALIBRATION_OFFSET_A2 - 0.05},    \
                 {CALIBRATION_OFFSET_A0, CALIBRATION_OFFSET_A1 - 0.05, -CALIBRATION_OFFSET_A2 - 0.05},    \
                 {CALIBRATION_OFFSET_A0, CALIBRATION_OFFSET_A1 - 0.05, -CALIBRATION_OFFSET_A2 - 0.05},    \
                 {CALIBRATION_OFFSET_A0, CALIBRATION_OFFSET_A1 - 0.05, -CALIBRATION_OFFSET_A2 - 0.05},    \
                 {CALIBRATION_OFFSET_A0, CALIBRATION_OFFSET_A1 - 0.05, -CALIBRATION_OFFSET_A2 - 0.05},    \
                 {CALIBRATION_OFFSET_A0, CALIBRATION_OFFSET_A1 - 0.05, -CALIBRATION_OFFSET_A2 - 0.05}}
#endif
