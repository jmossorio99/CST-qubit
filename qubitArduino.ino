/** Code for Assignment 1
* Qubit state representation using a tangible user interface
* @author José Ossorio V01010285
*/

// Setting pins for buttons to use quantum gates
int xBtnPin = 8;
int hBtnPin = 9;
int mBtnPin = 10;

int xLastBtnState = 1;
int hLastBtnState = 1;
int mLastBtnState = 1;

// Struct to group the attributes of a qubit. A qubit is described by a statevector with two amplitudes: ampKet0 and ampKet1
// Each of the kets has a led pin and an interval that determines when the respective led will blink.
// The higher the amplitude, the faster it will blink
struct qubit {
  int pinKet0;
  int pinKet1;
  int pinPhase;
  byte ledState0;
  byte ledState1;
  double ampKet0;
  double ampKet1;
  double intervalKet0;
  double intervalKet1;
};

// variables used to track time of the last blink for each ket
unsigned long previousMillis0 = 0;
unsigned long previousMillis1 = 0;

/** Code snippet taken from https://forum.arduino.cc/t/increasing-probability-with-a-number/358459/5
* Biased random probability for collapsing the state of the qubit based on the amplitudes of each component.
* For example, if ket 1 has probability of 80%, the threshold will be 80 and the maximum will always be 100.
* That way, if we generate a random number between 0 and 100, 80% of the time it will be less than or equal to 100, returning true
* @param: threshold - probability bias
* @param: maximum - the maximum probability (always 100)
* @return: boolean indicating whether the generated number is below the threshold
*/
bool chance(int threshold, int maximum) {
  long ran = random(maximum);
  return ran <= threshold;
}

/**
* Helper function to calculate the probability of collapsing the qubit to a specific state based on the amplitude of that state.
* The probability is calculated by squaring the absolute value of the amplitude.
* @param: amplitude - the amplitude of a component of the qubit
* @return: double - the probability
*/
double prob(double amplitude) {
  return abs(amplitude) * abs(amplitude);
}

/**
* Helper function to calculte the product of a 2x2 matrix and a 2-vector
* @param: m - the matrix
* @param: v - the vector
*/
void matrixByVector(double m[][2], double v[]) {
  double result[2] = { NULL, NULL };
  for (int i = 0; i < 2; i++) {
    for (int j = 0; j < 1; j++) {
      result[i] = (m[i][j] * v[j]) + (m[i][j + 1] * v[j + 1]);
    }
  }
  v[0] = result[0];
  v[1] = result[1];
}

/**
* Helper function to assign new amplitudes after applying a quantum gate
* @param: pQ - a pointer to a qubit
* @param: statevector - the new statevector to be assigned to the qubit
*/
void assignAmplitudes(struct qubit* pQ, double statevector[]) {
  pQ->ampKet0 = statevector[0];
  pQ->ampKet1 = statevector[1];
}

void H(struct qubit* pQ) {
  double H[2][2] = { { 1.0 / sqrt(2), 1.0 / sqrt(2) }, { 1.0 / sqrt(2), -1.0 / sqrt(2) } };
  double statevector[2] = { pQ->ampKet0, pQ->ampKet1 };
  // Check if the qubit is in state |1> and apply a negative phase to |1>
  matrixByVector(H, statevector);
  assignAmplitudes(pQ, statevector);
  previousMillis0 = 0;
  previousMillis1 = 0;
}

void X(struct qubit* pQ) {
  double X[2][2] = { { 0.0, 1.0 }, { 1.0, 0.0 } };
  double statevector[2] = { pQ->ampKet0, pQ->ampKet1 };
  matrixByVector(X, statevector);
  assignAmplitudes(pQ, statevector);
  previousMillis0 = 0;
  previousMillis1 = 0;
}

void measure(struct qubit* pQ) {
  double statevector[2] = { 0.0, 0.0 };
  if (chance(prob(pQ->ampKet0) * 100, 100)) {
    statevector[0] = 1.0;
    statevector[1] = 0.0;
    assignAmplitudes(pQ, statevector);
  } else {
    statevector[0] = 0.0;
    statevector[1] = 1.0;
    assignAmplitudes(pQ, statevector);
  }
  previousMillis0 = 0;
  previousMillis1 = 0;
}

qubit q0 = { 2, 3, 4, LOW, LOW, 0.38, 0.92, -1, -1};
qubit* pQ0 = &q0;

void setup() {
  Serial.begin(9600);
  randomSeed(analogRead(A0));
  q0.intervalKet0 = prob(q0.ampKet0) == 0 ? -1.0 : (1000 - prob(q0.ampKet0) * 1000);
  q0.intervalKet1 = prob(q0.ampKet1) == 0 ? -1.0 : (1000 - prob(q0.ampKet1) * 1000);

  pinMode(q0.pinKet0, OUTPUT);
  pinMode(q0.pinKet1, OUTPUT);
  pinMode(q0.pinPhase, OUTPUT);

  pinMode(xBtnPin, INPUT_PULLUP);
  pinMode(hBtnPin, INPUT_PULLUP);
  pinMode(mBtnPin, INPUT_PULLUP);
}

void loop() {
  unsigned long currentMillis = millis();
  // Calculate interval for each LED
  q0.intervalKet0 = prob(q0.ampKet0) == 0 ? -1 : (1000 - prob(q0.ampKet0) * 1000);
  q0.intervalKet1 = prob(q0.ampKet1) == 0 ? -1 : (1000 - prob(q0.ampKet1) * 1000);
  
  // Check if we should blink the pin for Ket0
  if ((currentMillis - previousMillis0) > q0.intervalKet0) {
    previousMillis0 = currentMillis;
    if (q0.intervalKet0 == -1.0) {
      q0.ledState0 = LOW;
    } else {
      q0.ledState0 = (q0.ledState0 == HIGH) ? LOW : HIGH;
    }
  }

  // Check if we should blink the pin for Ket1
  if ((currentMillis - previousMillis1) > q0.intervalKet1) {
    previousMillis1 = currentMillis;
    if (q0.intervalKet1 == -1.0) {
      q0.ledState1 = LOW;
    } else {
      // Serial.println("changing ket 1");
      q0.ledState1 = (q0.ledState1 == HIGH) ? LOW : HIGH;
    }
  }
  digitalWrite(q0.pinKet0, q0.ledState0);
  digitalWrite(q0.pinKet1, q0.ledState1);

  int xBtnState = digitalRead(xBtnPin);
  int hBtnState = digitalRead(hBtnPin);
  int mBtnState = digitalRead(mBtnPin);

  // Button state change code taken from: https://docs.arduino.cc/built-in-examples/digital/StateChangeDetection
  // compare the buttonState to its previous state
  if (xBtnState != xLastBtnState) {
    if (xBtnState == LOW) {
      // if the current state is HIGH then the button went from off to on:
      X(pQ0);
    }
    // Delay a little bit to avoid bouncing
    delay(50);
  }
  // save the current state as the last state, for next time through the loop
  xLastBtnState = xBtnState;

  // compare the buttonState to its previous state
  if (hBtnState != hLastBtnState) {
    if (hBtnState == LOW) {
      // if the current state is HIGH then the button went from off to on:
      H(pQ0);
    }
    // Delay a little bit to avoid bouncing
    delay(50);
  }
  // save the current state as the last state, for next time through the loop
  hLastBtnState = hBtnState;

  // compare the buttonState to its previous state
  if (mBtnState != mLastBtnState) {
    if (mBtnState == HIGH) {
      // if the current state is HIGH then the button went from off to on:
      measure(pQ0);
    }
    // Delay a little bit to avoid bouncing
    delay(50);
  }
  // save the current state as the last state, for next time through the loop
  mLastBtnState = mBtnState;

  if (q0.ampKet0 < 0 || q0.ampKet1 < 0) {
    digitalWrite(q0.pinPhase, HIGH);
  } else {
    digitalWrite(q0.pinPhase, LOW);
  }
}
