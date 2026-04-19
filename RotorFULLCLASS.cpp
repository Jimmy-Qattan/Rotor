#include <vector>

class Rotor {
    
    uint8_t PIN;
    float POSITION;
    Servo SERVO;
    String NAME;
    bool inMotion = false;
    bool FORCED_STOP = false;
    bool GRACE_STOP = false;
    int SPEED = 1000; // Determined as Time

    float stepSize = 0.0f;

    bool sequenced = false;
    bool cueFull = false;
    int seqIndex, seqIndexCUE = 0;
    vector<float>seqPositions = {0.0f, 180.0f};

    void runSequence() {
        sequenced = true;
        // seqIndex = 0; ? (by choice)
    };

    bool isSequenced() const {
        return sequenced;
    };

    int currentSequenceIndex() const {
        return seqIndex;
    };

    void setSequenceIndex(int value) {
        if (inMotion) {
            seqIndexCUE = value;
        };
    };

    void runSequence() {
        
    };
    
    bool hasCue = false;
    float CUE;
    int CUESPEED;
    
    int currentGrace = 0;
    int finalGrace = 1000;
    
    float FINALPOSITION;

    float INITIALPOSITIONSTAG; // Will be useful during the math in order to keep initial and final positions stagnant while current position is moving through its motion interpolation. Ideal for linear/non-linear velocities.
    
    bool INITIALIZED = false;
    
    static inline int NUM_OF_SERVOS = 0;
    static inline int NUM_OF_WORKING_SERVOS = 0;

    public:
    
        Rotor(int pin, String name) : PIN((uint8_t)pin), NAME(name), INITIALIZED(true) {
            SERVO.attach(pin);
            
            POSITION = INITIALPOSITIONSTAG = FINALPOSITION = (float)(SERVO.read());

            NUM_OF_WORKING_SERVOS++;
        };
        
        Rotor(int pin) : PIN((uint8_t)pin), NAME(String("Robot " + String(NUM_OF_SERVOS++))), INITIALIZED(true) {
            SERVO.attach(pin);
            
            POSITION = INITIALPOSITIONSTAG = FINALPOSITION = (float)(SERVO.read());
            
            NUM_OF_WORKING_SERVOS++;
        };
        
        Rotor() : PIN(0), NAME(String("Robot " + String(NUM_OF_SERVOS++))), POSITION((float)(0)), INITIALIZED(false) {};

        int pin() const {
            return (int)PIN;
        };
        
        float position() const {
            return POSITION;
        };
        
        void setPin(int pin) {
            if (INITIALIZED) {
                SERVO.detach();
            } else {
                INITIALIZED = true;
                NUM_OF_WORKING_SERVOS++;
                NUM_OF_SERVOS++;
            }
            
            SERVO.attach(pin);
        }
        
        void removePin() {
            if (!INITIALIZED) return;
            
            SERVO.detach();
            NUM_OF_WORKING_SERVOS--;
        }
        
        void setName(String name) {
            NAME = name;
        };
        
        String getName() const {
            return NAME;
        };
        
        void setSpeed(int speed) {
            SPEED = speed;
        }
        
        int getSpeed() const {
            return SPEED;
        }
        
        // For when addValue gets called too many times in loop
        
        void setFinalGrace(int value) {
            finalGrace = value;    
        }
        
        void setCurrentGrace(int value) {
            currentGrace = value;    
        };
        
        int getFinalGrace() const {
            return finalGrace;
        };
        
        int getCurrentGrace() const {
            return currentGrace;
        };
        
        void beginGrace() {
            currentGrace = 0;
        }
        
        // TICK

        void constrainPosition(bool ROUND = false) {
            POSITION = constrain(POSITION, 0, 180);
            
            if (FINALPOSITION > INITIALPOSITIONSTAG) {
                POSITION = constrain(POSITION, INITIALPOSITIONSTAG, FINALPOSITION);
            } else {
                POSITION = constrain(POSITION, FINALPOSITION, INITIALPOSITIONSTAG);
            }

            if (ROUND) {
                POSITION = round(POSITION);
            }
        }
        
        void addValue(float value) {
            POSITION += value;
            
            constrainPosition();
            
            SERVO.write(round(POSITION));
            
            inMotion = true;
        };
        
        void checkCue() {
            if (abs(FINALPOSITION - POSITION) <= abs(stepSize)) {                
                if (hasCue) {
                    writeSpeed(CUE, CUESPEED);
                    removeCue();
                }
            }
            else {
                beginGrace();
            }
        }
        
        // STOP AND RESUME FUNCTIONS CRUCIAL
        
        void STOP() {
            FORCED_STOP = true;
        }
        
        void RESUME() {
            FORCED_STOP = false;
        }
        
        void createCue(float value, int speed) {
            if (hasCue) return;
            
            hasCue = true;
            CUE = value;
            CUESPEED = speed;
        }
        
        void removeCue() {
            if (!hasCue) return;
            
            hasCue = false;
            CUE = 0.0f;
            CUESPEED = 0;
        }
        
        void executeCue() {
            if (!hasCue || inMotion) return;
            
            writeSpeed(CUE, CUESPEED);
            removeCue();
        }
        
        void writeSpeed(float value, int speed) {
            if (inMotion) {
                createCue(value, speed);
                return;
            };
            
            INITIALPOSITIONSTAG = POSITION;
            FINALPOSITION = value;
            SPEED = speed;

            stepSize = (FINALPOSITION - INITIALPOSITIONSTAG) / SPEED;
        }

        void checkToZeroStepSize() {
            if (inMotion) return;

            stepSize = 0.0f;
        }

        void PROCEDURAL_FINAL_CHECK() {
            if (inMotion && (abs(FINALPOSITION - POSITION) <= abs(stepSize))) {
                POSITION = FINALPOSITION;
                SERVO.write(round(POSITION));
                inMotion = false;
                checkToZeroStepSize();
                checkCue();
                return;
            }
        }

        float getStepSize() const {
            return stepSize;
        };
        
        void tick() {
            
            if (currentGrace < finalGrace) {
                GRACE_STOP = true;
                currentGrace++;
            } else {
                GRACE_STOP = false;
                setCurrentGrace(finalGrace);
            }

            PROCEDURAL_FINAL_CHECK();
            
            if (!SPEED || SPEED < 0) {
                SPEED = 1000;
            };
            
            
            
            // ADD SERVO CURRENTPOSITION BY PARTIAL
            
            if (FORCED_STOP || GRACE_STOP || !INITIALIZED || !inMotion) return; // IF SERVO IS FORCED STOP OR NOT INITIALIZED OR ON GRACE, NOTHING SHOULD HAPPEN
            
            addValue(stepSize);
        }
};
