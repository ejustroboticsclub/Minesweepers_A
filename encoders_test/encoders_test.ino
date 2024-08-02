#define CLK 20  // Pin connected to CLK pin of the encoder
#define DT 21   // Pin connected to DT pin of the encoder

int counter = 0; 
int currentStateCLK;
int previousStateCLK;

void setup() {
  // Set encoder pins as inputs
  pinMode(CLK, INPUT);
  pinMode(DT, INPUT);

  // Initialize serial communication
  Serial.begin(9600);

  // Read the initial state of CLK
  previousStateCLK = digitalRead(CLK);
}

void loop() {
  // Read the current state of CLK
  currentStateCLK = digitalRead(CLK);

  // If the state of CLK has changed, then update the counter
  if (currentStateCLK != previousStateCLK) {
    // Read the DT pin
    int dtState = digitalRead(DT);

    // Determine the direction of rotation
    if (dtState != currentStateCLK) {
      counter++;
    } else {
      counter--;
    }

    // Print the counter value
    Serial.print("Counter: ");
    Serial.println(counter);
  }

  // Update the previous state of CLK
  previousStateCLK = currentStateCLK;
}
