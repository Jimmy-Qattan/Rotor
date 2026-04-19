class Rotor {
    
    uint8_t PIN;
    float POSITION;
    Servo SERVO;
    String NAME;
    bool inMotion = false;
    bool FORCED_STOP = false;
    bool GRACE_STOP = false;
    int SPEED = 1000; // Determined as Time
    
    int currentGrace = 0; // Grace period refers to the amount of time after a movement in which the servo "rests"
    int finalGrace = 1000;
    
    float FINALPOSITION;
    
    bool INITIALIZED = false;
    
    static inline int NUM_OF_SERVOS = 0;
    static inline int NUM_OF_WORKING_SERVOS = 0;

    public:
    
        Rotor(int pin, String name) : PIN((uint8_t)pin), NAME(name), INITIALIZED(true) {
            SERVO.attach(pin);
            
            POSITION = (float)(SERVO.read());
            FINALPOSITION = POSITION;
            
            NUM_OF_WORKING_SERVOS++;
        };
        
        Rotor(int pin) : PIN((uint8_t)pin), NAME(String("Robot " + String(NUM_OF_SERVOS++))), INITIALIZED(true) {
            SERVO.attach(pin);
            
            POSITION = (float)(SERVO.read());
            FINALPOSITION = POSITION;
            
            NUM_OF_WORKING_SERVOS++;
        };
        
        Rotor() : PIN(0), NAME(String("Robot " + String(NUM_OF_SERVOS++))), POSITION((float)(0)), INITIALIZED(false) {};

        int pin() const;
        float position() const;
        void setPin(int pin);
        void removePin();
        void setName(String name);
        String getName() const;
        void setSpeed(int speed); // Setting the time it will take to arrive at positions via "writeSpeed"
        int getSpeed() const;
        void setFinalGrace(int value); 
        void setCurrentGrace(int value);
        int getFinalGrace() const;
        int getCurrentGrace() const;
        void beginGrace(); // Essentially tells servo to test for a bit of time
        void addValue(float value, int speed = SPEED); // Writes value to the servo
        float determineDifferential();
        void STOP();
        void RESUME();
        void writeSpeed(float value); // User uses this to write values to the servo
        void tick(); // MANDATORY FOR CONSTANT UPDATES
};
