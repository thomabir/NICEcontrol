#include "Controllers.hpp"

PIController::PIController() {
    p = 0;
    i = 0;
    error_integral = 0;
}

void PIController::setPI(float p, float i) {
    this->p = p;
    this->i = i;
}

void PIController::reset_state() {
    error_integral = 0;
}

float PIController::step(float input) {
    error_integral += input;
    return p * input + i * error_integral;
}