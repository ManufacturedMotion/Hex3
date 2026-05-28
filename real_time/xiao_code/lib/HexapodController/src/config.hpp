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
    #define ZERO_POINTS {{.441786 + CALIBRATION_OFFSET_A0, -2.419088 + CALIBRATION_OFFSET_A1, 1.362175 + CALIBRATION_OFFSET_A2}, {-2.239612 + CALIBRATION_OFFSET_A0,-2.822525 + CALIBRATION_OFFSET_A1,0.161068 + CALIBRATION_OFFSET_A2}, {2.534136 + CALIBRATION_OFFSET_A0,-0.633534 + CALIBRATION_OFFSET_A1,2.142971 + CALIBRATION_OFFSET_A2}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}}
    #define MIN_POS {{-M_PI / 3.0, -M_PI / 2.0, -M_PI / 2.0}, {-M_PI, -M_PI, -M_PI}, {-M_PI, -M_PI, -M_PI}, {-M_PI, -M_PI, -M_PI}, {-M_PI, -M_PI, -M_PI}, {-M_PI, -M_PI, -M_PI}}
    #define MAX_POS {{M_PI / 3.0, M_PI / 2.0, M_PI / 2.0}, {M_PI, M_PI, M_PI}, {M_PI, M_PI, M_PI}, {M_PI, M_PI, M_PI}, {M_PI, M_PI, M_PI}, {M_PI, M_PI, M_PI}}
    #define SCALE_FACT {{1, 1, 1}, {1, 1, 1}, {1, 1, 1}, {1, 1, 1}, {1, 1, 1}, {1, 1, 1}}
    #define REVERSE_AXIS {{false, true, true}, {false, true, true}, {false, true, true}, {false, false, false}, {false, false, false}, {false, false, false}}
#endif

#ifdef DILLON
    #define STEP_THRESHOLD 40 
	#define FIFO_IDLE_THRESHOLD 100
    #define ZERO_POINTS {                                    \
    {                                                    \
        0.281229 + CALIBRATION_OFFSET_A0,                \
        -2.711151 + CALIBRATION_OFFSET_A1,                \
        -0.037259 + CALIBRATION_OFFSET_A2                 \
    },                                                  \
    {                                                    \
        0 + CALIBRATION_OFFSET_A0,                       \
        -2.878246 + CALIBRATION_OFFSET_A1,                \
        2.622118 + CALIBRATION_OFFSET_A2                 \
    },                                                  \
    {                                                    \
        0.278161 + CALIBRATION_OFFSET_A0,                \
        0.116085 + CALIBRATION_OFFSET_A1,                \
        1.137225 + CALIBRATION_OFFSET_A2                 \
    },                                                  \
    {                                                    \
        0.089482 + CALIBRATION_OFFSET_A0,                \
        1.323328 + CALIBRATION_OFFSET_A1,                \
        -0.502601 + CALIBRATION_OFFSET_A2                 \
    },                                         \
    {                                                    \
        1.080433 + CALIBRATION_OFFSET_A0,                \
        -1.652595 + CALIBRATION_OFFSET_A1,                \
        0.282797 + CALIBRATION_OFFSET_A2                 \
    },                                            \
    {                                                    \
        0 + CALIBRATION_OFFSET_A0,                \
        0 + CALIBRATION_OFFSET_A1,                \
        0 + CALIBRATION_OFFSET_A2                 \
    },                                          \
    };
    #define MIN_POS {{-M_PI / 3.0, -M_PI / 2.0, -1.8}, {-M_PI, -M_PI, -M_PI}, {-M_PI, -M_PI, -M_PI}, {-M_PI, -M_PI, -M_PI}, {-M_PI, -M_PI, -M_PI}, {-M_PI, -M_PI, -M_PI}}
    #define MAX_POS {{M_PI / 3.0, M_PI / 2.0, 1.8}, {M_PI, M_PI, M_PI}, {M_PI, M_PI, M_PI}, {M_PI, M_PI, M_PI}, {M_PI, M_PI, M_PI}, {M_PI, M_PI, M_PI}}
    #define SCALE_FACT {{1, 1, 1}, {1, 1, 1}, {1, 1, 1}, {1, 1, 1}, {1, 1, 1}, {1, 1, 1}}
    #define REVERSE_AXIS {{true, false, false}, {true, false, false}, {true, false, false}, {false, false, false}, {true, false, false}, {false, false, false}}
#endif

#ifdef DANNY
    #define STEP_THRESHOLD 40 
	#define FIFO_IDLE_THRESHOLD 100
    #define ZERO_POINTS {{-2.247282, 1.793224, -0.846757}, {0.323670, 2.155243, 0.316000}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}}
    #define MIN_POS {{-M_PI / 3.0, -M_PI / 2.0, -1.8}, {-M_PI, -M_PI, -M_PI}, {-M_PI, -M_PI, -M_PI}, {-M_PI, -M_PI, -M_PI}, {-M_PI, -M_PI, -M_PI}, {-M_PI, -M_PI, -M_PI}}
    #define MAX_POS {{M_PI / 3.0, M_PI / 2.0, 1.8}, {M_PI, M_PI, M_PI}, {M_PI, M_PI, M_PI}, {M_PI, M_PI, M_PI}, {M_PI, M_PI, M_PI}, {M_PI, M_PI, M_PI}}
    #define SCALE_FACT {{1, 1, 1}, {1, 1, 1}, {1, 1, 1}, {1, 1, 1}, {1, 1, 1}, {1, 1, 1}}
    #define REVERSE_AXIS {{true, false, false}, {false, false, false}, {false, false, false}, {false, false, false}, {false, false, false}, {false, false, false}}
#endif
#endif