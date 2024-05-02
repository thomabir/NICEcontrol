class PIController {
 public:
  PIController();
  void setPI(float p, float i);
  void reset_state();
  void reset_all();
  float step(float input);

 private:
  float p;
  float i;
  float error_integral;
};