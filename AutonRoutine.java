package org.firstinspires.ftc.teamcode.auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Sensor;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.Claw;
import org.firstinspires.ftc.teamcode.hardware.Arm;
import org.firstinspires.ftc.robotcore.external.Telemetry;

@Autonomous(name="AutonRoutine")
public class AutonRoutine extends OpMode {

    // Declare robot components
    private Robot robot; // Robot class that handles hardware
    private Sensor huskySensor; // Husky sensor
    private DistanceSensor distanceSensor; // Distance sensor
    private Claw claw; // Claw mechanism
    private Arm arm; // Arm mechanism

    @Override
    public void init() {
        // Initialize robot components
        robot = new Robot(hardwareMap);
        huskySensor = hardwareMap.get(Sensor.class, "huskySensor");
        distanceSensor = hardwareMap.get(DistanceSensor.class, "distanceSensor");
        claw = new Claw(hardwareMap);
        arm = new Arm(hardwareMap);

        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void runOpMode() {
        // Wait for the start button to be pressed
        waitForStart();

        // Determine which autonomous routine to run
        if (isNeutralAuton()) {
            runNeutralAuton();
        } else {
            runAllianceSpecificAuton();
        }
    }


    // probably wont be necessary
    private boolean isNeutralAuton() {
        // Logic to determine if Neutral Auton is selected
        HuskyLens.Block[] blocks = huskyLens.blocks();
        telemetry.addData("Block count", blocks.length);
        for (int i = 0; i < blocks.length; i++) {
            telemetry.addData("Block: ", blocks[i].toString());
            String color = "";
            if (blocks[i].id == 2) {
                return true;  // Neutral autonomously if block ID matches the neutral condition
            }
        }
        return false;
    }

    private void runNeutralAuton() {
        // Step 1: Preload Specimen
        preloadSpecimen();

        // Step 2-5: Repeat for 3 blocks
        for (int i = 0; i < 3; i++) {
            // Step 2: Move robot towards backstage parking
            moveToBackstageParking();

            // Step 3: Extend Claw to upper basket
            extendClawToUpperBasket();

            // Step 4: Drop Yellow Block in upper basket
            dropYellowBlock();

            // Step 5: Utilize Husky Sensor and Distance sensor to calculate distance
            calculateDistanceForPlacedBlocks();
        }

        // Step 6: Backstage Parking
        moveToBackstageParking();
    }

    private void runAllianceSpecificAuton() {
        // Step 1: Preload 1 specimen
        preloadSpecimen();

        // Step 2: Drive to the submersible and hang it at the highest bar possible
        robot.driveForward(1.0);  // Move forward to the submersible
        arm.raise();  // Raise the arm to hang the specimen
        robot.driveForward(0.5);  // Move forward slightly to position the specimen
        dropSpecimen();  // Drop the specimen

        // Step 3: Move back towards human zone
        robot.driveBackward(1.0);  // Move back towards the human zone

        // Step 4: Extend claw and pick up specimen
        claw.extend();  // Extend claw to pick up the specimen
        claw.close();   // Close claw to grab the specimen

        // Step 5: Turn around and let go of sample on submersible
        robot.turn(0.5);  // Turn the robot around
        dropSpecimen();  // Drop the specimen on the submersible

        // Step 6: Park in backstage area
        moveToBackstageParking();

        // Optional: If time permits, take 1 sample and put in higher basket
        if (timePermits()) {
            moveToBackstageParking(); // Move to the parking area
            extendClawToUpperBasket(); // Extend claw to upper basket
            dropYellowBlock(); // Drop the block in the upper basket
        }

        // Final parking
        moveToBackstageParking();
    }

    private void preloadSpecimen() {
        // Code to preload the specimen
        arm.raise(); // Raise arm to take in block
        claw.open(); // Open claw to preload
    }

    private void moveToBackstageParking() {
        // Code to move the robot towards backstage parking
        robot.driveForward(1.0); // Drive forward for a specified time or distance
    }

    private void extendClawToUpperBasket() {
        // Code to extend the claw to the upper basket
        arm.raise(); // Raise arm up
        
    }

    private void dropYellowBlock() {
        // Code to drop the yellow block
        claw.close(); // Close claw to drop the block
    }

    private void calculateDistanceForPlacedBlocks() {
        // Code to utilize sensors to calculate distance
        double distance = distanceSensor.getDistance(DistanceUnit.CM); // Get distance in cm
        telemetry.addData("Distance to Block", distance);
    }

    private void dropSpecimen() {
        // Code to drop the specimen
        arm.lower();  // Lower the arm to drop the specimen
        claw.open();  // Open claw to release the specimen
    }

    private boolean timePermits() {
        // Example check for time remaining (you can replace this with actual timing logic)
        return getRuntime() < 10;  // If the runtime is less than 10 seconds, it's time to do the next step
    }
}
