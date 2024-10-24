class PIController {
 public:
  PIController() {
    p = 0;
    i = 0;
    error_integral = 0;
  }
  void setPI(float p, float i) {
    this->p = p;
    this->i = i;
  }
  void reset_state() { error_integral = 0; }
  void reset_all() {
    error_integral = 0;
    p = 0;
    i = 0;
  }
  float step(float input) {
    error_integral += input;
    return p * input + i * error_integral;
  }

 private:
  float p;
  float i;
  float error_integral;
};