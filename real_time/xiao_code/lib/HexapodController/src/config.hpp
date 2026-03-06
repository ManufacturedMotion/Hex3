#include "axis.hpp"
#include <stdbool.h>
#include "user_config.hpp"

#ifndef HEXA_CONFIG
#define HEXA_CONFIG


#ifdef ZACK
	#define STEP_THRESHOLD 40 
	#define FIFO_IDLE_THRESHOLD 100
    #define ZERO_POINTS {{-0.378893, -1.432738, -2.259554}, {0.55292, 2.408350, -1.262466}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}}
    #define MIN_POS {{-M_PI / 3.0, -M_PI / 2.0, -1.8}, {-M_PI, -M_PI, -M_PI}, {-M_PI, -M_PI, -M_PI}, {-M_PI, -M_PI, -M_PI}, {-M_PI, -M_PI, -M_PI}, {-M_PI, -M_PI, -M_PI}}
    #define MAX_POS {{M_PI / 3.0, M_PI / 2.0, 1.8}, {M_PI, M_PI, M_PI}, {M_PI, M_PI, M_PI}, {M_PI, M_PI, M_PI}, {M_PI, M_PI, M_PI}, {M_PI, M_PI, M_PI}}
    #define SCALE_FACT {{1, 1, 1}, {1, 1, 1}, {1, 1, 1}, {1, 1, 1}, {1, 1, 1}, {1, 1, 1}}
    #define REVERSE_AXIS {{true, false, false}, {false, true, true}, {false, false, false}, {false, false, false}, {false, false, false}, {false, false, false}}
#endif

#ifdef DILLON
    #define STEP_THRESHOLD 40 
	#define FIFO_IDLE_THRESHOLD 100
    #define ZERO_POINTS {{-0.635068, -0.199418, -2.853204}, {-1.434272, -0.147262, 0.044485}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}}
    #define MIN_POS {{-M_PI / 3.0, -M_PI / 2.0, -1.8}, {-M_PI, -M_PI, -M_PI}, {-M_PI, -M_PI, -M_PI}, {-M_PI, -M_PI, -M_PI}, {-M_PI, -M_PI, -M_PI}, {-M_PI, -M_PI, -M_PI}}
    #define MAX_POS {{M_PI / 3.0, M_PI / 2.0, 1.8}, {M_PI, M_PI, M_PI}, {M_PI, M_PI, M_PI}, {M_PI, M_PI, M_PI}, {M_PI, M_PI, M_PI}, {M_PI, M_PI, M_PI}}
    #define SCALE_FACT {{1, 1, 1}, {1, 1, 1}, {1, 1, 1}, {1, 1, 1}, {1, 1, 1}, {1, 1, 1}}
    #define REVERSE_AXIS {{true, false, false}, {true, false, false}, {false, false, false}, {false, false, false}, {false, false, false}, {false, false, false}}
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