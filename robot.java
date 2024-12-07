package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Robot {

    private DcMotor leftMotor;
    private DcMotor rightMotor;

    // Constructor to initialize the robot hardware
    public Robot(HardwareMap hardwareMap) {
        // Initialize motors
        leftMotor = hardwareMap.get(DcMotor.class, "leftMotor");
        rightMotor = hardwareMap.get(DcMotor.class, "rightMotor");

        // Reverse the right motor 
        rightMotor.setDirection(DcMotor.Direction.REVERSE);
    }

    // Method to drive the robot forward for a specified distance (in inches)
    public void driveForward(double inches) {
        // Define the encoder counts per inch (adjust for your robot's configuration)
        final double COUNTS_PER_INCH = 1440;  // Example: Adjust based on your wheel and motor setup

        // Reset encoders before starting movement
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Set the motors to use encoders to reach the target position
        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Calculate the target position based on the distance (in inches) and encoder counts per inch
        int targetPosition = (int) (inches * COUNTS_PER_INCH);

        // Set the target position for both motors
        leftMotor.setTargetPosition(targetPosition);
        rightMotor.setTargetPosition(targetPosition);

        // Set motor power to move the robot
        leftMotor.setPower(0.5);  // Adjust power as needed
        rightMotor.setPower(0.5);

        // Wait until both motors reach their target positions
        while (leftMotor.isBusy() && rightMotor.isBusy()) {
            // You can add telemetry here to track progress if desired
            telemetry.addData("Left Position", leftMotor.getCurrentPosition());
            telemetry.addData("Right Position", rightMotor.getCurrentPosition());
            telemetry.update();
        }

        // Stop the motors once the target position is reached
        leftMotor.setPower(0);
        rightMotor.setPower(0);

        // Reset motor modes to RUN_USING_ENCODER for future movements
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    //similar to moveforward function but multiply by -1 to reverse direction
    public void driveBackward(double inches) {
        final double COUNTS_PER_INCH = 1440;  

        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        int targetPosition = (int) (-inches * COUNTS_PER_INCH);  

        
        leftMotor.setTargetPosition(targetPosition);
        rightMotor.setTargetPosition(targetPosition);

        
        leftMotor.setPower(0.5);  
        rightMotor.setPower(0.5);

        
        while (leftMotor.isBusy() && rightMotor.isBusy()) {
            
            telemetry.addData("Left Position", leftMotor.getCurrentPosition());
            telemetry.addData("Right Position", rightMotor.getCurrentPosition());
            telemetry.update();
        }

       
        leftMotor.setPower(0);
        rightMotor.setPower(0);

        
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    // Method to turn the robot by setting the motors to opposite powers
    public void turn(double power) {
        leftMotor.setPower(power);  // Set power to the left motor
        rightMotor.setPower(-power); // Set power to the right motor (opposite direction for turning)
    }

    // Method to stop the robot
    public void stop() {
        leftMotor.setPower(0);  // Stop the left motor
        rightMotor.setPower(0); // Stop the right motor
    }
}
