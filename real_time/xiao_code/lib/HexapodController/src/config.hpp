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
            0.437185  + CALIBRATION_OFFSET_A0,  \
            -0.852893 + CALIBRATION_OFFSET_A1,  \
            1.362175  + CALIBRATION_OFFSET_A2}, \
         {\
            -2.218136 + CALIBRATION_OFFSET_A0, \
            -2.822525 + CALIBRATION_OFFSET_A1, \
            0.161068  + CALIBRATION_OFFSET_A2}, \
         {\
            -0.932660 + CALIBRATION_OFFSET_A0, \
            0.365087  + CALIBRATION_OFFSET_A1,  \
            -2.994330 + CALIBRATION_OFFSET_A2},\
         {\
            2.451301  + CALIBRATION_OFFSET_A0,  \
            0.897379  + CALIBRATION_OFFSET_A1,  \
            2.142971  + CALIBRATION_OFFSET_A2}, \
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
            -1.716014 + CALIBRATION_OFFSET_A0,     \
            -2.683430 + CALIBRATION_OFFSET_A1,     \
            -0.508737 + CALIBRATION_OFFSET_A2},    \
         {                                         \
            -1.372402 + CALIBRATION_OFFSET_A0,     \
            -0.201449 + CALIBRATION_OFFSET_A1,     \
            -0.573164 + CALIBRATION_OFFSET_A2},    \
         {                                         \
            1.060492 + CALIBRATION_OFFSET_A0,     \
            -1.856614 + CALIBRATION_OFFSET_A1,     \
            2.715691 + CALIBRATION_OFFSET_A2},    \
         {                                         \
            0.066472 + CALIBRATION_OFFSET_A0,     \
            0.294027 + CALIBRATION_OFFSET_A1,     \
            -2.335708 + CALIBRATION_OFFSET_A2},    \
         {                                         \
            0.37787 + CALIBRATION_OFFSET_A0,     \
            1.064085 + CALIBRATION_OFFSET_A1,     \
            1.434817 + CALIBRATION_OFFSET_A2},    \
         {                                         \
            0.308841 + CALIBRATION_OFFSET_A0,     \
            -2.447197 + CALIBRATION_OFFSET_A1,     \
            0.822759 + CALIBRATION_OFFSET_A2},    \
    }
    #define SCALE_FACT {{1, 1, 1}, {1, 1, 1}, {1, 1, 1}, {1, 1, 1}, {1, 1, 1}, {1, 1, 1}}
    #define TOE_IDLE_READ (float[]){0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f} 
    #define REVERSE_AXIS {{true, false, false}, {true, false, false}, {true, false, false}, {true, false, false}, {true, false, false}, {true, false, false}}
#endif
 
#ifdef DANNY
   #define STEP_THRESHOLD 40
   #define FIFO_IDLE_THRESHOLD 100
   #define ZERO_POINTS {                    \
      {0.709210 + CALIBRATION_OFFSET_A0,   \
      -0.930090 + CALIBRATION_OFFSET_A1,  \
      2.787788 + CALIBRATION_OFFSET_A2},  \
      {1.385185 + CALIBRATION_OFFSET_A0,   \
      -2.913029 + CALIBRATION_OFFSET_A1,  \
      -3.120117 + CALIBRATION_OFFSET_A2}, \
      {0.308330 + CALIBRATION_OFFSET_A0,   \
      0.023010 + CALIBRATION_OFFSET_A1,   \
      1.842311 + CALIBRATION_OFFSET_A2},  \
      {-2.098486 + CALIBRATION_OFFSET_A0,  \
      2.412952 + CALIBRATION_OFFSET_A1,   \
      2.586292 + CALIBRATION_OFFSET_A2},  \
      {1.394389 + CALIBRATION_OFFSET_A0,   \
      -1.667437 + CALIBRATION_OFFSET_A1,  \
      2.376136 + CALIBRATION_OFFSET_A2},  \
      {2.434427 + CALIBRATION_OFFSET_A0,   \
      -0.892777 + CALIBRATION_OFFSET_A1,  \
      -1.036971 + CALIBRATION_OFFSET_A2}, \
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
