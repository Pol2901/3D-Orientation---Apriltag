float q0, q1, q2, q3;
float q0_filt,q1_filt,q2_filt,q3_filt;
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
        q0_filt = a; q1_filt = b; q2_filt = c1; q3_filt = d;

        float norm = sqrt(q0_filt*q0_filt
                + q1_filt*q1_filt
                + q2_filt*q2_filt
                + q3_filt*q3_filt);
        
        q0 = q0_filt / norm;
        q1 = q1_filt / norm;
        q2 = q2_filt / norm;
        q3 = q3_filt / norm;


        

        bool isZeroFrame = (q0 == 0.0 && q1 == 0.0 && q2 == 0.0 && q3 == 0.0);

        if (isZeroFrame) {
          justCameBackFromZero = true;
          Serial.println("Quaternion: 0.0, 0.0, 0.0, 0.0");
        } else if (justCameBackFromZero) {
          Serial.println("[INFO] First valid frame after tag reacquisition ignored.");
          justCameBackFromZero = false;
        } else {
          Serial.print("Quaternion: ");
          Serial.print(q0,6); Serial.print(", ");
          Serial.print(q1,6); Serial.print(", ");
          Serial.print(q2,6); Serial.print(", ");
          Serial.println(q3,6);
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





