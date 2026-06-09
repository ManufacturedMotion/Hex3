#ifndef HEXA_TOE
#define HEXA_TOE

    #define TOE_PIN D0  ///< Pin for toe/ground contact sensor
    class Toe {
        public:
            Toe();
            float read();
        private:
    };

#endif