float q0, q1, q2, q3;
bool justCameBackFromZero = false;
String inputLine = "";

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("Bridge started.");

  Serial1.begin(115200);
  delay(2000);
  Serial1.println("RUN_SCRIPT");
}

void loop() {
  while (Serial1.available()) {
    char c = Serial1.read();

    if (c == '\n') {
      // End of line: process the complete message
      inputLine.trim();  // Remove leading/trailing spaces
      inputLine.replace("[", "");
      inputLine.replace("]", "");

      float a, b, c1, d;
      if (sscanf(inputLine.c_str(), "%f %f %f %f", &a, &b, &c1, &d) == 4) {
        q0 = a; q1 = b; q2 = c1; q3 = d;

        bool isZeroFrame = (q0 == 0.0 && q1 == 0.0 && q2 == 0.0 && q3 == 0.0);

        if (isZeroFrame) {
          justCameBackFromZero = true;
          Serial.println("Quaternion: 0.0, 0.0, 0.0, 0.0");
        } else if (justCameBackFromZero) {
          Serial.println("[INFO] First valid frame after tag reacquisition ignored.");
          justCameBackFromZero = false;
        } else {
          Serial.print("Quaternion: ");
          Serial.print(q0); Serial.print(", ");
          Serial.print(q1); Serial.print(", ");
          Serial.print(q2); Serial.print(", ");
          Serial.println(q3);
        }
      } else {
        Serial.println("[WARN] Malformed frame ignored: " + inputLine);
      }

      inputLine = ""; // Reset for next line
    } else {
      inputLine += c;  // Build the line character by character
    }
  }
}


