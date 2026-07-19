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
            -2.749917 + CALIBRATION_OFFSET_A0,     \
            -0.965372 + CALIBRATION_OFFSET_A1,     \
            -2.756018 + CALIBRATION_OFFSET_A2},    \
         {                                         \
            0.043462 + CALIBRATION_OFFSET_A0,      \
            -1.766109 + CALIBRATION_OFFSET_A1,     \
            0.759866 + CALIBRATION_OFFSET_A2},     \
         {                                         \
            0.348724 + CALIBRATION_OFFSET_A0,      \
            0.198920 + CALIBRATION_OFFSET_A1,      \
            1.043652 + CALIBRATION_OFFSET_A2},     \
         {                                         \
            1.799870 + CALIBRATION_OFFSET_A0,      \
            -2.832226 + CALIBRATION_OFFSET_A1,     \
            2.703419 + CALIBRATION_OFFSET_A2},     \
         {                                         \
            1.108045 + CALIBRATION_OFFSET_A0,      \
            -1.677139 + CALIBRATION_OFFSET_A1,     \
            0.319613 + CALIBRATION_OFFSET_A2},     \
         {                                         \
            0.391676 + CALIBRATION_OFFSET_A0,      \
            1.284978 + CALIBRATION_OFFSET_A1,      \
            -0.580834 + CALIBRATION_OFFSET_A2}     \
    }
    #define SCALE_FACT {{1, 1, 1}, {1, 1, 1}, {1, 1, 1}, {1, 1, 1}, {1, 1, 1}, {1, 1, 1}}
    #define TOE_IDLE_READ (float[]){26.7f, 19.0f, 25.5f, 29.3f, 28.32f, 26.4f} 
    #define REVERSE_AXIS {{true, false, false}, {true, false, false}, {true, false, false}, {true, false, false}, {true, false, false}, {true, false, false}}
#endif

#ifdef DANNY
   #define STEP_THRESHOLD 40
   #define FIFO_IDLE_THRESHOLD 100
   #define ZERO_POINTS {                    \
      {0.613592 + CALIBRATION_OFFSET_A0,   \
      -0.952602 + CALIBRATION_OFFSET_A1,  \
      2.804117 + CALIBRATION_OFFSET_A2},  \
      {2.443631 + CALIBRATION_OFFSET_A0,   \
      -0.898913 + CALIBRATION_OFFSET_A1,  \
      -1.032369 + CALIBRATION_OFFSET_A2}, \
      {1.316156 + CALIBRATION_OFFSET_A0,   \
      -1.790156 + CALIBRATION_OFFSET_A1,  \
      2.337787 + CALIBRATION_OFFSET_A2}, \
      {-2.332674 + CALIBRATION_OFFSET_A0,   \
      2.267224 + CALIBRATION_OFFSET_A1,  \
      2.597029 + CALIBRATION_OFFSET_A2}, \
      {0.325204 + CALIBRATION_OFFSET_A0,   \
      -0.070563 + CALIBRATION_OFFSET_A1,  \
      1.816233 + CALIBRATION_OFFSET_A2}, \
      {1.448078 + CALIBRATION_OFFSET_A0,   \
      -2.900758 + CALIBRATION_OFFSET_A1,  \
      -3.112447 + CALIBRATION_OFFSET_A2}, \
   }
   #define SCALE_FACT {{1, 1, 1}, {1, 1, 1}, {1, 1, 1}, {1, 1, 1}, {1, 1, 1}, {1, 1, 1}}
   #define TOE_IDLE_READ (float[]){0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f} 
   #define REVERSE_AXIS                                                                                                                       \
      {                                                                                                                                       \
         {true, false, false}, {true, false, false}, {true, false, true}, {true, false, false}, {true, false, false}, { true, false, false } \
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
