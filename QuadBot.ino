#include <Arduino.h>
#include <Servo.h>
#include <IRrecv.h>
#include <IRutils.h>

// --- Pin mapping (NodeMCU D-pins) ---
constexpr uint8_t IR_RECEIVE_PIN = D3;

// Front Left
constexpr uint8_t FL_ROT_PIN = D1;
constexpr uint8_t FL_LIFT_PIN = D0;

// Back Left
constexpr uint8_t BL_ROT_PIN = D4;
constexpr uint8_t BL_LIFT_PIN = D2;

// Front Right
constexpr uint8_t FR_ROT_PIN = D6;
constexpr uint8_t FR_LIFT_PIN = D5;

// Back Right
constexpr uint8_t BR_ROT_PIN = D7;
constexpr uint8_t BR_LIFT_PIN = D8;
IRrecv irrecv(IR_RECEIVE_PIN);
decode_results results;

// --- Servo objects ---
Servo flRot, flLift;
Servo blRot, blLift;
Servo frRot, frLift;
Servo brRot, brLift;

// --- Home positions (degrees) ---
// Start with 90 everywhere, then tune per-servo.
// If a servo is mechanically centered differently, change its HOME value.
int FL_ROT_HOME = 90;
int FL_LIFT_HOME = 90;

int BL_ROT_HOME = 90;
int BL_LIFT_HOME = 90;

int FR_ROT_HOME = 90;
int FR_LIFT_HOME = 90;

int BR_ROT_HOME = 90;
int BR_LIFT_HOME = 90;

// Optional: clamp helper
int clampDeg(int deg) {
	if (deg < 0) return 0;
	if (deg > 180) return 180;
	return deg;
}

void attachAllServos() {
	// Typical servo pulse range; adjust if needed.
	// attach(pin, minPulse, maxPulse)
	flRot.attach(FL_ROT_PIN, 500, 2500);
	flLift.attach(FL_LIFT_PIN, 500, 2500);

	blRot.attach(BL_ROT_PIN, 500, 2500);
	blLift.attach(BL_LIFT_PIN, 500, 2500);

	frRot.attach(FR_ROT_PIN, 500, 2500);
	frLift.attach(FR_LIFT_PIN, 500, 2500);

	brRot.attach(BR_ROT_PIN, 500, 2500);
	brLift.attach(BR_LIFT_PIN, 500, 2500);
}

void moveAllToHome() {
	flRot.write(clampDeg(FL_ROT_HOME));
	flLift.write(clampDeg(FL_LIFT_HOME));

	blRot.write(clampDeg(BL_ROT_HOME));
	blLift.write(clampDeg(BL_LIFT_HOME));

	frRot.write(clampDeg(FR_ROT_HOME));
	frLift.write(clampDeg(FR_LIFT_HOME));

	brRot.write(clampDeg(BR_ROT_HOME));
	brLift.write(clampDeg(BR_LIFT_HOME));
}

void detachAllServos() {
	flRot.detach();
	flLift.detach();
	blRot.detach();
	blLift.detach();
	frRot.detach();
	frLift.detach();
	brRot.detach();
	brLift.detach();
}

void printMapping() {
	Serial.println();
	Serial.println("QuadBot reset sketch");
	Serial.println("Leg pin mapping:");
	Serial.println("  Front Left : rot D1, lift D0");
	Serial.println("  Back  Left : rot D4, lift D2");
	Serial.println("  Front Right: rot D6, lift D5");
	Serial.println("  Back  Right: rot D7, lift D8");
	Serial.println();
	Serial.println("Sending all servos to HOME positions...");
}

void frontRightLegForward(float phase) {
	const int ROT_CENTER = 90;  // neutral hip angle
	const int ROT_AMP = 25;     // how far forward/back
	const int LIFT_DOWN = 80;   // foot on ground (tune)
	const int LIFT_UP = 130;    // foot lifted (tune)

	float rot = ROT_CENTER + ROT_AMP * sinf(phase);

	float swing = cosf(phase);  // +1..-1
	float lift;
	if (swing > 0.0f) {
		float u = swing;                     // 0..1
		float smooth = u * u * (3 - 2 * u);  // smoothstep
		lift = LIFT_DOWN + (LIFT_UP - LIFT_DOWN) * smooth;
	} else {
		lift = LIFT_DOWN;  // stance/push phase: keep foot down
	}

	frRot.write(clampDeg((int)roundf(rot)));
	frLift.write(clampDeg((int)roundf(lift)));
}

void frontLeftLegForward(float phase) {
	const int ROT_CENTER = 90;  // neutral hip angle
	const int ROT_AMP = 25;     // how far forward/back
	const int LIFT_DOWN = 80;   // foot on ground (tune)
	const int LIFT_UP = 130;    // foot lifted (tune)

	float rot = ROT_CENTER + ROT_AMP * sinf(phase);

	float swing = cosf(phase);  // +1..-1
	float lift;
	if (swing > 0.0f) {
		float u = swing;                     // 0..1
		float smooth = u * u * (3 - 2 * u);  // smoothstep
		lift = LIFT_DOWN + (LIFT_UP - LIFT_DOWN) * smooth;
	} else {
		lift = LIFT_DOWN;  // stance/push phase: keep foot down
	}

	flRot.write(clampDeg((int)roundf(rot)));
	flLift.write(clampDeg((int)roundf(lift)));
}
void backRightLegForward(float phase) {
	const int ROT_CENTER = 90;
	const int ROT_AMP = 25;

	const int LIFT_DOWN = 90;  // tune
	const int LIFT_UP = 20;    // tune (or invert if needed)

	phase += PI;

	float rot = ROT_CENTER + ROT_AMP * sinf(phase);

	float swing = cosf(phase);  // +1..-1
	float lift;

	if (swing > 0.0f) {
		// swing phase: lift
		float u = swing;                     // 0..1
		float smooth = u * u * (3 - 2 * u);  // smoothstep
		lift = LIFT_DOWN + (LIFT_UP - LIFT_DOWN) * smooth;
	} else {
		// stance phase: keep down
		lift = LIFT_DOWN;
	}

	brRot.write(clampDeg((int)roundf(rot)));
	brLift.write(clampDeg((int)roundf(lift)));
}
void backLeftLegForward(float phase) {
	const int ROT_CENTER = 90;
	const int ROT_AMP = 25;

	const int LIFT_DOWN = 90;  // tune
	const int LIFT_UP = 20;    // tune (or invert if needed)

	phase += PI;

	float rot = ROT_CENTER + ROT_AMP * sinf(phase);

	float swing = cosf(phase);  // +1..-1
	float lift;

	if (swing > 0.0f) {
		// swing phase: lift
		float u = swing;                     // 0..1
		float smooth = u * u * (3 - 2 * u);  // smoothstep
		lift = LIFT_DOWN + (LIFT_UP - LIFT_DOWN) * smooth;
	} else {
		// stance phase: keep down
		lift = LIFT_DOWN;
	}

	blRot.write(clampDeg((int)roundf(rot)));
	blLift.write(clampDeg((int)roundf(lift)));
}
void forward() {
	const float SPEED = 2.0f;
	float t = millis() * 0.001f;
	float phase = t * SPEED;

	frontRightLegForward(phase);
	backRightLegForward(phase);
	// frontLeftLegForward(phase);
	// backLeftLegForward(phase);
}
void control(int msg) {
	Serial.print("0x");
	Serial.println(msg, HEX);

	switch (msg) {
		case 0xFF18E7:
			{
				Serial.println("Forward");
				float s = sin(millis() * 0.001f);  // millis → seconds
				float val = (s + 1.0f) * 90.0f;    // -1..1 → 0..180
				int _deg = round(val);

				frRot.write(clampDeg(_deg));
				// frLift
				break;
			}
		case 0xFF4AB5:
			Serial.println("Backward");
			break;
		case 0xFF10EF:
			Serial.println("Left");
			break;
		case 0xFF5AA5:
			Serial.println("Right");
			break;
		default:
			break;
	}
	Serial.println("--------------------");
}
// void controler()
void setup() {
	Serial.begin(115200);
	delay(200);

	irrecv.enableIRIn();
	printMapping();

	attachAllServos();
	delay(200);

	moveAllToHome();

	// Give time to physically move
	delay(1500);

	Serial.println("Done. (Servos remain attached.)");
	Serial.println("Tip: send 'r' over Serial to reset again, 'd' to detach, 'a' to attach.");

	// If you prefer to detach automatically after reset, uncomment:
	// detachAllServos();
}

void loop() {
	// Simple serial controls (optional but handy)
	if (irrecv.decode(&results)) {
		control(results.value);
		irrecv.resume();  // receive the next value
	}
	if (Serial.available()) {
		char c = (char)Serial.read();
		if (c == 'r' || c == 'R') {
			Serial.println("Reset -> HOME");
			moveAllToHome();
		} else if (c == 'd' || c == 'D') {
			Serial.println("Detaching all servos");
			detachAllServos();
		} else if (c == 'a' || c == 'A') {
			Serial.println("Attaching all servos");
			attachAllServos();
		}
	}
	forward();
}
