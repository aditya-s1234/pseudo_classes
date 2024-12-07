package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Arm {

    private Servo armServo;  // Servo controlling the arm

    // Constructor to initialize the arm class
    public Arm(HardwareMap hardwareMap) {
        armServo = hardwareMap.get(Servo.class, "armServo");
    }

    // Method to raise the arm
    public void raise() {
        armServo.setPosition(1.0); // Assuming 1.0 is the raised position
    }

    // Method to lower the arm
    public void lower() {
        armServo.setPosition(0.0); // Assuming 0.0 is the lowered position
    }
}
