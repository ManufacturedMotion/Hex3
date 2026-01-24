#include "axis.hpp"
#include <stdbool.h>
#include "user_config.hpp"

#ifndef HEXA_CONFIG
#define HEXA_CONFIG

	#define STEP_THRESHOLD 40 
	#define FIFO_IDLE_THRESHOLD 100
    #define ZERO_POINTS {{-2.687534, -0.745515, 0.845223}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}}
    #define MIN_POS {{-M_PI / 3.0, -M_PI / 2.0, -1.8}, {-M_PI, -M_PI, -M_PI}, {-M_PI, -M_PI, -M_PI}, {-M_PI, -M_PI, -M_PI}, {-M_PI, -M_PI, -M_PI}, {-M_PI, -M_PI, -M_PI}}
    #define MAX_POS {{M_PI / 3.0, M_PI / 2.0, 1.8}, {M_PI, M_PI, M_PI}, {M_PI, M_PI, M_PI}, {M_PI, M_PI, M_PI}, {M_PI, M_PI, M_PI}, {M_PI, M_PI, M_PI}}
    #define SCALE_FACT {{1, 1, 1}, {1, 1, 1}, {1, 1, 1}, {1, 1, 1}, {1, 1, 1}, {1, 1, 1}}
    #define REVERSE_AXIS {{true, true, false}, {false, false, false}, {false, false, false}, {false, false, false}, {false, false, false}, {false, false, false}}

#endif