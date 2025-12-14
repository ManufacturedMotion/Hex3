#include "axis.hpp"
#include <stdbool.h>
#include "user_config.hpp"

#ifndef HEXA_CONFIG
#define HEXA_CONFIG

	#define STEP_THRESHOLD 40 
	#define FIFO_IDLE_THRESHOLD 100
    #define ZERO_POINTS {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}}
    #define MIN_POS {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}}
    #define MAX_POS {{2 * M_PI, 2 * M_PI, 2 * M_PI}, {2 * M_PI, 2 * M_PI, 2 * M_PI}, {2 * M_PI, 2 * M_PI, 2 * M_PI}, {2 * M_PI, 2 * M_PI, 2 * M_PI}, {2 * M_PI, 2 * M_PI, 2 * M_PI}, {2 * M_PI, 2 * M_PI, 2 * M_PI}}
    #define SCALE_FACT {{1, 1, 1}, {1, 1, 1}, {1, 1, 1}, {1, 1, 1}, {1, 1, 1}, {1, 1, 1}}
    #define REVERSE_AXIS {{false, false, false}, {false, false, false}, {false, false, false}, {false, false, false}, {false, false, false}, {false, false, false}}

#endif