package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Claw {

    private Servo clawServo; // Servo controlling the claw

    // Constructor to initialize the claw class
    public Claw(HardwareMap hardwareMap) {
        clawServo = hardwareMap.get(Servo.class, "clawServo");
    }

    // Method to open the claw
    public void open() {
        clawServo.setPosition(0.0); // Assuming 0.0 is the open position
    }

    // Method to close the claw
    public void close() {
        clawServo.setPosition(1.0); // Assuming 1.0 is the closed position
    }

    // Method to extend the claw
    public void extend() {
        clawServo.setPosition(0.5); // Assuming 0.5 is the extended position (adjust as needed)
    }
}
