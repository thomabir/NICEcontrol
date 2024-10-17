// a sketch for how to realise the control loops in NICEcontrol.

// top-level overview:
// Actuator: init, move
// Sensor: init, read
// Controller:
//    member: Sensor(s), Actuator(s)
//    init
//    control: read sensors, get setpoint, compute and return control signal

// minimal classes:
class Actuator {
 public:
  void init();
  void move(float control_signal);
};

class Sensor {
 public:
  void init();
  float read();
};

class Controller {
 public:
  Controller(Sensor *sensor, Actuator *actuator);
  void init();
  float control();

 private:
  Sensor *sensor;
  Actuator *actuator;
};

// A specific thing to implement: characterise a control loop by applying a dither signal to the control variable, and
// measuring the response. Do this for multiple frequencies, and store the data as csv files in the format `10-hz.csv`,
// `20-hz.csv`, etc.

// A sketch of how to implement this:

void characterise_loop(Controller *controller, float freq) {
  // set up the dither signal
  float amplitude = 0.1;
  float period = 1.0 / freq;
  float t = 0;
  float control_signal = 0;
  // open file
  char filename[20];
  sprintf(filename, "%d-hz.csv", freq);
  FILE *f = fopen(filename, "w");
