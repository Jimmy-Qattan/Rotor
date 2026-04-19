#include "Rotor.h"

int Rotor::pin() const {
    return (int)PIN;
};

float Rotor::position() const {
    return POSITION;
};

void Rotor::setPin(int pin) {
    if (INITIALIZED) {
        SERVO.detach();
    } else {
        INITIALIZED = true;
        NUM_OF_WORKING_SERVOS++;
        NUM_OF_SERVOS++;
    }
    
    SERVO.attach(pin);
}

void Rotor::removePin() {
    if (!INITIALIZED) return;
    
    SERVO.detach();
    NUM_OF_WORKING_SERVOS--;
}

void Rotor::setName(String name) {
    NAME = name;
};

String Rotor::getName() const {
    return NAME;
};

void Rotor::setSpeed(int speed) {
    SPEED = speed;
}

int Rotor::getSpeed() const {
    return SPEED;
}

// For when addValue gets called too many times in loop

void Rotor::setFinalGrace(int value) {
    finalGrace = value;    
}

void Rotor::setCurrentGrace(int value) {
    currentGrace = value;    
};

int Rotor::getFinalGrace() const {
    return finalGrace;
};

int Rotor::getCurrentGrace() const {
    return currentGrace;
};

void Rotor::beginGrace() {
    currentGrace = 0;
}

// TICK

void Rotor::addValue(float value) {
    POSITION += value;
    
    POSITION = constrain(POSITION, 0, 180);
    SERVO.write(round(POSITION));
    
    inMotion = true;
    
    
};

float Rotor::determineDifferential() {
    if (abs(FINALPOSITION - POSITION) <= 0.01) beginGrace();
    
    return ((FINALPOSITION - POSITION) / SPEED);
}

// STOP AND RESUME FUNCTIONS CRUCIAL

void Rotor::STOP() {
    FORCED_STOP = true;
}

void Rotor::RESUME() {
    FORCED_STOP = false;
}

// Real real fun stuff

void Rotor::writeSpeed(float value, int speed = SPEED) {
    FINALPOSITION = value;
    SPEED = speed;
}

void Rotor::tick() {
    
    if (currentGrace < finalGrace) {
        GRACE_STOP = true;
        currentGrace++;
    } else {
        GRACE_STOP = false;
        setCurrentGrace(finalGrace);
    }
    
    if (POSITION == FINALPOSITION) {
        inMotion = false;
    }
    
    if (!SPEED || SPEED < 0) {
        SPEED = 1000;
    };
    
    // ADD SERVO CURRENTPOSITION BY PARTIAL
    
    if (FORCED_STOP || GRACE_STOP) return; // IF SERVO IS FORCED STOP, NOTHING SHOULD HAPPEN
    
    addValue(determineDifferential());
    
}
