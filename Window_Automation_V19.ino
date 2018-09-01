#include <Wire.h>
#include <DS3231.h>
#include <Adafruit_INA219.h>

DS3231 clock;
RTCDateTime dt;

Adafruit_INA219 ina219;

#define IN1 2
#define IN2 3
#define IN3 4
#define IN4 5
#define ENB 6
#define EndStop_closed 7
#define EndStop_open 8
#define Relay 9
#define BT_state 12

// Analog in
#define TempSensin 0
#define TempSensout 1
#define MoistureSens 2

short bluetoothBuffer = 0, bluetoothReceive = 0;
int unlockedDurationLimitOpening = 5000, unlockedDurationLimitClosing = 1000, unlockedDurationBuffer = 3000, stoppedDurationLimit = 5000, minimumOpenDuration = 180, minimumClosedDuration = 600, maximumClosedDuration = 14400;
int W_open = 0, W_closed = 1, W_opening = 0, W_closing = 0, W_stopped = 0, W_state, W_manualOpening = 0, W_manualClosing = 0, W_cooling = 0, W_heating = 0, W_rain = 0, W_timeClosing = 0, W_timeOpeningShort = 0, W_timeOpeningLong = 0;
int stopCurrent = 900, stoppedOpening = 0, stoppedClosing = 0, stoppedOpeningTimeWrong = 0, stopCountedThisCycle = 0, stoppedCounter = 0;
int timerOpen = 0, timerClosed = 1, endStopOpenStatus = 0, endStopClosedStatus = 0, locked = 1, unlockedThisCycle = 0, timestampRecordedThisCycle = 0, firstClosing = 1;
int ignoreRain = 0, ignoreLock = 0, ignoreHeating = 0, ignoreCooling = 0, ignoreTimer = 0, ignoreEndStops = 0;
int tempSet = 22, tempReadingin = 0, tempReadingout = 0, tempReadingCounter = 0, tempReadingLimit = 100, tempReadingIndex = 0;
int rainDelay = 10, norainMoisture = 100, rainMoisture = 200, moistureReading = 0, dryness = 0, moistureTotal = 0, moistureAverage = 0, moistureReadingCounter = 0, moistureReadingLimit = 50, moistureReadingIndex = 0;
int currentReading = 0, currentTotal = 0, currentAverage = 0, currentReadingCounter = 0, currentReadingLimit = 20, currentReadingIndex = 0;
int tempCin[100], tempCout[100], moisture[50], current[20];
long timestampOpen = 0, timestampClosed = 0, timestampRain = 0, timestampUnlocked = 0, timestampStopped = 0, timestampClosingStart = 0, timestampOpeningStart = 0, closingSpeed = 0, openingSpeed = 0, lockedDurationLimit = 0;
float currentVpp = 0.0048828; //5V / 1024 analog measuring steps
float current_mA = 0;
float R1 = 10000;
float R2in = 0, R2out = 0, logR2in = 0, logR2out = 0, tempKin = 0, tempKout = 0, tempCinTotal = 0, tempCoutTotal = 0, tempCinAverage = 0, tempCoutAverage = 0, currentVoltage = 0;
float A = 1.013525350e-03, B = 2.546035115e-04, C = -0.05751721517e-07;

void setup() {
  // Initialize DS3231
  Serial.begin(9600);

  clock.begin();

  pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(Relay, OUTPUT);

  pinMode(EndStop_closed, INPUT);
  pinMode(EndStop_open, INPUT);
  pinMode(BT_state, INPUT);

  digitalWrite(ENB, LOW);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);

  for (int i = 0; i < tempReadingLimit; i++) {
    tempCin[i] = 0;
    tempCout[i] = 0;
  }
  for (int i = 0; i < moistureReadingLimit; i++) {
    moisture[i] = 0;
  }

  clock.setDateTime(__DATE__, __TIME__);
  dt = clock.getDateTime();
  timestampClosed = dt.unixtime - minimumClosedDuration + 20;     // Window cannot open automatically for 20 seconds after the system start
  ina219.begin();
}

void loop() {
  dt = clock.getDateTime();
  endStopOpenStatus = digitalRead(EndStop_open);
  endStopClosedStatus = digitalRead(EndStop_closed);

  //Check if 2 bytes are available
  if (Serial.available() >= 2) {
    bluetoothBuffer = 0;
    
    //Save the available bytes in a buffer by using bit shifting
    for (int i = 0; i < 2; i++) {
      bluetoothBuffer |= Serial.read() << (8 * i);
    }
    
    //Validate the data
    if (bluetoothBuffer != 0) {
      bluetoothReceive = bluetoothBuffer;

      //Set different parameters depending on the value received
      if (0 <= bluetoothReceive && bluetoothReceive <= 99) {
        switch (bluetoothReceive) {
          case 1:
            W_manualOpening = 1;
            W_manualClosing = 0;
            break;
          case 2:
            W_manualClosing = 1;
            W_manualOpening = 0;
            break;
          case 10:
            endStopOpenStatus = 1;
            endStopClosedStatus = 1;
            break;
          case 20:
            ignoreRain = 1;
            break;
          case 21:
            ignoreRain = 0;
            break;
          case 22:
            ignoreLock = 1;
            break;
          case 23:
            ignoreLock = 0;
            break;
          case 24:
            ignoreTimer = 1;
            break;
          case 25:
            ignoreTimer = 0;
            break;
          case 26:
            ignoreEndStops = 1;
            break;
          case 27:
            ignoreEndStops = 0;
            break;
          case 28:
            ignoreHeating = 1;
            break;
          case 29:
            ignoreHeating = 0;
            break;
          case 30:
            ignoreCooling = 1;
            break;
          case 31:
            ignoreCooling = 0;
            break;
          case 256:             // In case of faulty Bluetooth communication: Empty Serial input buffer by pressing Manual Opening in the app
            while (Serial.available() >= 1) {
              bluetoothBuffer = Serial.read();
              bluetoothBuffer = 0;
            }
        }
      }
      if (100 <= bluetoothReceive && bluetoothReceive <= 150) {
        tempSet = bluetoothReceive - 100;
      }
    }
  }

  if (tempReadingCounter < tempReadingLimit) {
    tempReadingCounter++;
  }
  else {
    tempReadingCounter = tempReadingLimit;
  }
  if (tempReadingIndex >= tempReadingLimit) {
    tempReadingIndex = 0;
  }
  tempReadingin = analogRead(TempSensin);
  R2in = R1 * (1023.0 / (float)tempReadingin - 1.0);
  logR2in = log(R2in);
  tempKin = (1.0 / (A + B * logR2in + C * logR2in * logR2in * logR2in));
  tempCinTotal = tempCinTotal - tempCin[tempReadingIndex];
  tempCin[tempReadingIndex] = 100 * (tempKin - 273.15);         // Conversion from float to int
  tempCinTotal = tempCinTotal + tempCin[tempReadingIndex];
  tempCinAverage = (tempCinTotal / tempReadingCounter) / 100;   // Conversion back to correct temperature

  tempReadingout = analogRead(TempSensout);
  R2out = R1 * (1023.0 / (float)tempReadingout - 1.0);
  logR2out = log(R2out);
  tempKout = (1.0 / (A + B * logR2out + C * logR2out * logR2out * logR2out));
  tempCoutTotal = tempCoutTotal - tempCout[tempReadingIndex];
  tempCout[tempReadingIndex] = 100 * (tempKout - 273.15);
  tempCoutTotal = tempCoutTotal + tempCout[tempReadingIndex];
  tempCoutAverage = (tempCoutTotal / tempReadingCounter) / 100;

  tempReadingIndex++;

  moistureReading = analogRead(MoistureSens); // Current measuring doesn't work because the sensor output is far too unstable. Analog value should be 512 when currentless, but it's usually at 400 and jumps by up to 50. ACS712 board potentially broken.
  dryness = moistureReading;

  if (moistureReadingCounter < moistureReadingLimit) {
    moistureReadingCounter++;
  }
  else {
    moistureReadingCounter = moistureReadingLimit;
  }
  if (moistureReadingIndex >= moistureReadingLimit) {
    moistureReadingIndex = 0;
  }
  moistureTotal = moistureTotal - moisture[moistureReadingIndex];
  moisture[moistureReadingIndex] = 1023 - dryness;
  moistureTotal = moistureTotal + moisture[moistureReadingIndex];
  moistureAverage = moistureTotal / moistureReadingCounter;
  moistureReadingIndex++;

  currentReading = ina219.getCurrent_mA();
  if (currentReadingCounter < currentReadingLimit) {
    currentReadingCounter++;
  }
  else {
    currentReadingCounter = currentReadingLimit;
  }
  if (currentReadingIndex >= currentReadingLimit) {
    currentReadingIndex = 0;
  }
  currentTotal = currentTotal - current[currentReadingIndex];
  current[currentReadingIndex] = currentReading;
  currentTotal = currentTotal + current[currentReadingIndex];
  currentAverage = currentTotal / currentReadingCounter;
  currentReadingIndex++;

  currentVoltage = currentAverage * currentVpp - 2.5; // Measured analog value * Volts per point - Offset to 0 V

  Serial.print("|1");
  Serial.print(tempSet);
  Serial.print("|2");
  Serial.print(tempCinAverage);
  Serial.print("|3");
  Serial.print(tempCoutAverage);
  Serial.print("|4");
  Serial.print(moistureAverage);
  Serial.print("|5");
  Serial.print(bluetoothBuffer);
  Serial.print("|6");
  Serial.print(W_state);
  Serial.print("|7");
  Serial.print(currentReading);
  Serial.print("|8");
  Serial.print("Parameter 8");

  if (ignoreRain == 0) {
    if (moistureAverage >= rainMoisture) {
      W_rain = 1;
      timestampRain = dt.unixtime;
    }
    if (W_rain == 1) {
      if (moistureAverage <= norainMoisture) {
        long rainDuration = dt.unixtime - timestampRain;
        if (rainDuration >= rainDelay) {
          W_rain = 0;
        }
        if (ignoreTimer == 1) {
          W_rain = 0;
        }
      }
    }
  }
  if (ignoreRain == 1) {
    W_rain = 0;
  }

  if (locked == 0) {
    if (ignoreLock == 0) {
      digitalWrite(IN1, HIGH);
      digitalWrite(IN2, LOW);
    }
    if (stoppedCounter == 0) {
      // Only lock if it is not the first closing or if 
      if ((firstClosing == 0) || (stoppedOpeningTimeWrong == 0)) {
        long unlockedDuration = millis() - timestampUnlocked;
        if (W_opening == 1) {
          if (unlockedDuration >= unlockedDurationLimitOpening) {
            locked = 1;
          }
        }
      }
    }
    if (W_closed == 1 || W_open == 1) {
      locked = 1;
    }
  }
  if (locked == 1) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
  }

  if (ignoreEndStops == 1) {
    endStopOpenStatus = 1;
    endStopClosedStatus = 1;
  }

  int diffSetTemp = tempCinAverage - tempSet;
  if (diffSetTemp <= -2) {
    if (tempCinAverage < tempCoutAverage) {
      if (ignoreHeating == 0) {
        W_heating = 1;
      }
    }
  }
  if (W_heating == 1) {
    if (tempCinAverage > tempCoutAverage) {
      W_heating = 0;
    }
    if (diffSetTemp >= 1) {
      W_heating = 0;
    }
  }
  if (ignoreHeating == 1) {
    W_heating = 0;
  }

  if (diffSetTemp >= 2) {
    if (tempCinAverage > tempCoutAverage) {
      if (ignoreCooling == 0) {
        W_cooling = 1;
      }
    }
  }
  if (W_cooling == 1) {
    if (tempCinAverage < tempCoutAverage) {
      W_cooling = 0;
    }
    if (diffSetTemp <= -1) {
      W_cooling = 0;
    }
  }
  if (ignoreCooling == 1) {
    W_cooling = 0;
  }

  if (ignoreTimer == 1) {
    W_timeClosing = 1;
    W_timeOpeningShort = 1;
  }

  if (timerOpen == 1) {
    if (ignoreTimer == 0) {
      long openDuration = dt.unixtime - timestampOpen;
      if (openDuration >= minimumOpenDuration) {
        W_timeClosing = 1;
      }
      else {
        W_timeClosing = 0;
      }
    }
  }

  if (timerClosed == 1) {
    if (ignoreTimer == 0) {
      if (W_closed == 1) {
        long closedDuration = dt.unixtime - timestampClosed;
        if (closedDuration >= minimumClosedDuration) {
          W_timeOpeningShort = 1;
        }
        else {
          W_timeOpeningShort = 0;
        }
        if (closedDuration >= maximumClosedDuration) {
          W_timeOpeningLong = 1;
        }
        else {
          W_timeOpeningLong = 0;
        }
      }
    }
  }

  if (digitalRead(BT_state) == 0) {
    W_opening = 0;
    W_open = 0;
    W_manualClosing = 0;
    W_manualOpening = 0;
    if(W_closed == 0) {
      if(W_stopped == 0) {
        
        W_closing = 1;
        
      }
    }
  }

  if (W_closed == 1) {
    if (W_manualOpening == 1) {
      W_opening = 1;
    }
    if (W_manualClosing == 1) {
      W_manualClosing = 0;
    }
    else {
      if (digitalRead(BT_state) == 1) {
        if (W_rain == 0) {
          if (W_timeOpeningShort == 1) {
            if (W_cooling == 1) {
              W_opening = 1;
            }
            if (W_heating == 1) {
              W_opening = 1;
            }
          }
          if (W_timeOpeningLong == 1) {
            W_opening = 1;
          }
        }
      }
    }
    if (W_opening == 1) {
      W_closed = 0;
    }
    else {
      W_state = 0;
      W_open = 0;
      W_closing = 0;
      W_opening = 0;
    }
  }

  if (W_open == 1) {
    if (W_manualOpening == 1) {
      W_manualOpening = 0;
    }
    else {
      if (W_rain == 1) {
        W_closing = 1;
      }
      if (W_timeClosing == 1) {
        if (W_cooling == 0) {
          if (W_heating == 0) {
            W_closing = 1;
          }
        }
      }
    }
    if (W_manualClosing == 1) {
      W_closing = 1;
    }
    if (W_closing == 1) {
      W_open = 0;
    }
    else {
      W_state = 1;
      W_closed = 0;
      W_closing = 0;
      W_opening = 0;
    }
  }

  if (W_stopped == 1) {
    if (stopCountedThisCycle == 0) {
      stoppedCounter++;
      stopCountedThisCycle = 1;
    }
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
    digitalWrite(Relay, LOW);
    digitalWrite(ENB, LOW);
    locked = 1;
    if(stoppedCounter <= 2) {
      long stoppedDuration = millis() - timestampStopped;
      if (stoppedDuration >= stoppedDurationLimit) {
        if (stoppedClosing == 1) {
          W_stopped = 0;
          W_closing = 1;
          W_manualClosing = 0;
          stoppedClosing = 0;
        }
        if (stoppedOpening == 1) {
          W_stopped = 0;
          W_opening = 1;
          W_manualOpening = 0;
          stoppedOpening = 0;
        }
      }
    }
    if(stoppedCounter >= 3) {
      if (W_manualOpening == 1) {
        W_stopped = 0;
        W_opening = 1;
        W_manualOpening = 0;
      }
      if (W_manualClosing == 1) {
        W_stopped = 0;
        W_closing = 1;
        W_manualClosing = 0;
      }
    }
  }

  if (W_closing == 1) {
    if (timestampRecordedThisCycle == 0) {
      timestampClosingStart = millis();
      timestampRecordedThisCycle = 1;
    }
    if (unlockedThisCycle == 0) {
      long lockedDuration = millis() - timestampClosingStart;
      if (firstClosing == 1) {
        if (stoppedOpeningTimeWrong == 0) {
          lockedDurationLimit = openingSpeed - (unlockedDurationLimitClosing + unlockedDurationBuffer);
        }
        if (stoppedOpeningTimeWrong == 1) {
          locked = 0;
        }
      }
      if (firstClosing == 0) {
        lockedDurationLimit = closingSpeed - unlockedDurationLimitClosing;
      }
      if (lockedDuration >= lockedDurationLimit) {
        locked = 0;
        timestampUnlocked = millis();
        unlockedThisCycle = 1;
      }
    }
    if (stoppedCounter >= 1) {
      locked = 0;
    }
    if (W_manualOpening == 1) {
      W_closing = 0;
      W_opening = 1;
    }
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    digitalWrite(Relay, HIGH);
    digitalWrite(ENB, HIGH);
    W_state = 2;
    W_timeClosing = 0;
    timerOpen = 0;
    if (currentReading >= stopCurrent) {
      W_stopped = 1;
      W_closing = 0;
      stoppedClosing = 1;
      stopCountedThisCycle = 0;
      timestampStopped = millis();
    }
    if (endStopClosedStatus == 1) {
      digitalWrite(IN3, LOW);
      digitalWrite(IN4, LOW);
      digitalWrite(Relay, LOW);
      digitalWrite(ENB, LOW);
      W_closed = 1;
      W_closing = 0;
      timerClosed = 1;
      if(stoppedCounter == 0) {
        closingSpeed = millis() - timestampClosingStart;
      }
      timestampClosed = dt.unixtime;
      timestampRecordedThisCycle = 0;
      unlockedThisCycle = 0;
      firstClosing = 0;
      stoppedOpeningTimeWrong = 0;
      stoppedCounter = 0;
    }
  }

  if (W_opening == 1) {
    if (timestampRecordedThisCycle == 0) {
      timestampOpeningStart = millis();
      timestampRecordedThisCycle = 1;
    }
    if (unlockedThisCycle == 0) {
      locked = 0;
      timestampUnlocked = millis();
      unlockedThisCycle = 1;
    }
    if (stoppedCounter >= 1) {
      locked = 0;
    }
    if (W_manualClosing == 1) {
      W_closing = 1;
      W_opening = 0;
    }
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    digitalWrite(Relay, LOW);
    digitalWrite(ENB, HIGH);
    W_state = 3;
    W_timeOpeningShort = 0;
    W_timeOpeningLong = 0;
    timerClosed = 0;
    timerOpen = 1;
    if (currentReading >= stopCurrent) {
      W_stopped = 1;
      W_opening = 0;
      stoppedOpening = 1;
      stoppedOpeningTimeWrong = 1;
      stopCountedThisCycle = 0;
      timestampStopped = millis();
    }
    if (endStopOpenStatus == 1) {
      digitalWrite(IN3, LOW);
      digitalWrite(IN4, LOW);
      digitalWrite(Relay, LOW);
      digitalWrite(ENB, LOW);
      W_open = 1;
      W_opening = 0;
      openingSpeed = millis() - timestampOpeningStart;
      timestampOpen = dt.unixtime;
      timestampRecordedThisCycle = 0;
      unlockedThisCycle = 0;
      stoppedCounter = 0;
    }
  }

  if (W_closing == 0 && W_opening == 0) {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
    digitalWrite(Relay, LOW);
    digitalWrite(ENB, LOW);
  }
}
