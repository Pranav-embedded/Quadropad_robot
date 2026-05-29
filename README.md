
#   Quadruped Servo Robot – Arduino Nano Controlled

This project implements a **12-servo quadruped robot** using an **Arduino Nano** microcontroller. The robot features **realistic walking and movement patterns**, **Bluetooth control**

---

##  Features

-  **Basic Servo Initialization**: Positions all 12 servo motors to a neutral (90°) position.
-  **Autonomous Motion Sequences**: Performs walking (forward/backward), turning (left/right), and dance routines with coordinated leg motion.
-  **Bluetooth Control**: Accepts commands (F, B, L, R, U, W, V) over Bluetooth to trigger movement.
-  **Inverse Kinematics-Based Leg Control**: Each leg's endpoint is calculated and updated in real-time using a Cartesian-to-Polar transformation.

---

##  Bluetooth Command List

| Command | Action        |
|---------|---------------|
| `W`     | Step Forward  |
| `S`     | Step Backward |
| `A`     | Turn Left     |
| `D`     | Turn Right    |
| `U`     | Hand Shake    |
| `W`     | Hand Wave     |
| `V`     | Body Dance    |

---

##  Hardware Used

- Arduino Nano
- 12× SG90 (can use MG90S Servo Motors for more torque but power requirement will be more) 
- HC-05 Bluetooth Module
- Ultrasonic sensor for obstacle avoiding
- Power Source (e.g., 2× Li-ion 18650 batteries)
- Nano 328p Expansion board (to controll 12 servos)

---

##  Code Structure

- `setup()`: Initializes servos, sets default positions, and starts the servo service.
- `loop()`: Reads Bluetooth serial input and triggers appropriate movement functions.
- `motion functions`: `step_forward()`, `turn_left()`, `sit()`, `stand()` etc.
- `servo_service()`: Timer-based ISR that updates each servo position using inverse kinematics.

---

##  Getting Started

1. Upload the code to your Arduino using the Arduino IDE.
2. Power the servo motors using a separate power supply.
3. Pair your mobile device with the HC-05 module.
4. Use a Bluetooth terminal app to send commands (`W`, `S`, etc.).
5. Watch your quadruped robot come to life!


---

##  License

This project is open-source and available under the MIT License.

---

Made with ❤️ by [Pranav]
