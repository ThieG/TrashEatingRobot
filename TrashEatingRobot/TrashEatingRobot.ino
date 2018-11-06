/*!
****************************************************************************************************
* \file      TrashEatingRobot.ino
*
* \ingroup   ROBOT
*
*            Dieses Programm laesst den hungrigen Roboter essen.
*            Die Idee und Vorlage ist hier zu finden:
*            https://www.instructables.com/id/HUNGRY-ROBOT-Eating-Robot-3D-PRINTER-ARDUINO-SENSO/
*
****************************************************************************************************
*/
/*!
****************************************************************************************************
* \defgroup ROBOT     Hungriger Roboter 
****************************************************************************************************
*/
#include "SoftServo.h"  // SoftwareServo (works on non PWM pins)

/*------------------------------------------------------------------------------------------------*/
/*!
* \brief     Pin Konfiguration
*/
/*------------------------------------------------------------------------------------------------*/
#define ARM_SERVO_PIN     0   /* P0 */
#define AUGEN_ANALOG_PIN  A1  /* P2 */
#define ALIVE_LED_PIN     1   /* P1 -> OnBoard LED */
#define ALIVE_LED_INTERVAL_MS 250

/*------------------------------------------------------------------------------------------------*/
/*!
* \brief     Globale Variablen und Hadles
*/
/*------------------------------------------------------------------------------------------------*/
Adafruit_SoftServo armServo;
int letzterAugenSensorWert = 0;
const int AUGEN_THRESHOLD = 450;

typedef struct ServoSeqType {
  int Pos;
  int Delay;
} ServoSeqType;
ServoSeqType ServoSequence[] = {
  { 10, 300}, /* Schaufel nach oben zum Mund... */
  { 70, 600}, /* vor dem Bauch zittern ....     */
  { 50, 250},
  { 70, 250},
  { 50, 250},
  { 70, 250},
  { 50, 250},
  { 70, 250},
  { 50, 250},
  { 90, 250}, /* und wieder in Wartestellung gehen! */
  {  0,   0}    
};

/*------------------------------------------------------------------------------------------------*/
/*!
* \brief     Arduino Setup
*
*            Wird beim Start des Programms (entweder nach dem uebertragen auf das Board oder 
*            nach dem die Stromversorgung angeschlossen wurde) einmalig aufgerufen, 
*            um z. B. Pins als Eingang oder Ausgang zu definieren.
*/
/*------------------------------------------------------------------------------------------------*/
void setup() {

  // Timer 1
  noInterrupts();           // Alle Interrupts tempor�r abschalten
  OCR1A = 160; // compare match value 
  TIMSK |= (1 << OCIE1A); // enable compare match interrupt
  interrupts();             // alle Interrupts scharf schalten
  
  armServo.attach(ARM_SERVO_PIN);
  armServo.write(90);
  delay(1000);
  armServo.detach();

  pinMode(AUGEN_ANALOG_PIN, INPUT)  ;
  pinMode(ALIVE_LED_PIN, OUTPUT);
}

/*------------------------------------------------------------------------------------------------*/
/*!
* \brief     Arduino Main Loop
*
*            Wird durchgehend immer wieder durchlaufen, solange das Arduino-Board eingeschaltet ist.
*/
/*------------------------------------------------------------------------------------------------*/
void loop() {
  
  int augenSensorWert = analogRead(AUGEN_ANALOG_PIN);
  if (letzterAugenSensorWert <= AUGEN_THRESHOLD) {
    if (augenSensorWert > AUGEN_THRESHOLD) {
      action();
    } 
  }
  
  letzterAugenSensorWert = augenSensorWert;
  delay(10);

  blinkAliveLED();
}

/*------------------------------------------------------------------------------------------------*/
/*!
* \brief     Lebenskennzeichen geben
*
*            Hier wird die ALIVE LED bliken gelassen!
*/
/*------------------------------------------------------------------------------------------------*/
void blinkAliveLED() {

  static int aliveLEDState = LOW;
  static unsigned long previousMillis = 0;
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= ALIVE_LED_INTERVAL_MS) {
    // save the last time you blinked the LED
    previousMillis = currentMillis;

    // if the LED is off turn it on and vice-versa:
    if (aliveLEDState == LOW) {
      aliveLEDState = HIGH;
    } else {
      aliveLEDState = LOW;
    }

    // set the LED with the ledState of the variable:
    digitalWrite(ALIVE_LED_PIN, aliveLEDState);
  }
}

/*------------------------------------------------------------------------------------------------*/
/*!
* \brief     Fress Sequenz
*/
/*------------------------------------------------------------------------------------------------*/
void action() {
  //eating
  delay(1000);
  armServo.attach(ARM_SERVO_PIN);
  
  int curServoSeqIdx = 0;
  while (ServoSequence[curServoSeqIdx].Delay != 0) {
    armServo.write(ServoSequence[curServoSeqIdx].Pos);
    delay(ServoSequence[curServoSeqIdx].Delay);
    curServoSeqIdx++;
  }
  
  //release arm's torque
  armServo.detach();
}

/*------------------------------------------------------------------------------------------------*/
/*!
* \brief     Timer1 Compare ISR
*
*            Hier kommt die selbstdefinierte Interruptbehandlungsroutine 
*            f�r den Timer Compare Interrupt
*/
/*------------------------------------------------------------------------------------------------*/
volatile uint8_t counter = 0;
SIGNAL(TIMER1_COMPA_vect) {

  // this gets called every 2 milliseconds
  counter += 1;
  // every 20 milliseconds, refresh the servos!
  if (counter >= 20) {
    counter = 0;
    armServo.refresh();
  }
 
}
