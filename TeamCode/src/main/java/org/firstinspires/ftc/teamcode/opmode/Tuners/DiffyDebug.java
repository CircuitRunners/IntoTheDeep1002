package org.firstinspires.ftc.teamcode.opmode.Tuners; // Adjust the package name based on your project structure

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="DiffyDebug", group="TeleOp")
public class DiffyDebug extends OpMode {

    // Servo declarations
    private Servo diffy1;
    private Servo diffy2;

    // Servo positions initialized to 0.5
    private double diffy1Position = 0.5;
    private double diffy2Position = 0.5;

    // Previous state variables for edge detection
    private boolean prevLeftBumper = false;
    private boolean prevRightBumper = false;
    private boolean prevLeftTriggerPressed = false;
    private boolean prevRightTriggerPressed = false;

    @Override
    public void init() {
        // Map hardware
        diffy1 = hardwareMap.get(Servo.class, "diffy1");
        diffy2 = hardwareMap.get(Servo.class, "diffy2");

        // Set initial servo positions
        diffy1.setPosition(diffy1Position);
        diffy2.setPosition(diffy2Position);
    }

    @Override
    public void loop() {
        // Edge detection for left bumper (decrement diffy1)
        if (gamepad1.left_bumper && !prevLeftBumper) {
            diffy1Position -= 0.01;
        }

        // Edge detection for right bumper (increment diffy1)
        if (gamepad1.right_bumper && !prevRightBumper) {
            diffy1Position += 0.01;
        }

        // Determine if triggers are pressed (threshold > 0.5)
        boolean leftTriggerPressed = gamepad1.left_trigger > 0.5;
        boolean rightTriggerPressed = gamepad1.right_trigger > 0.5;

        // Edge detection for left trigger (decrement diffy2)
        if (leftTriggerPressed && !prevLeftTriggerPressed) {
            diffy2Position -= 0.01;
        }

        // Edge detection for right trigger (increment diffy2)
        if (rightTriggerPressed && !prevRightTriggerPressed) {
            diffy2Position += 0.01;
        }

        // Constrain servo positions to [0.0, 1.0]
        diffy1Position = Math.max(0.0, Math.min(1.0, diffy1Position));
        diffy2Position = Math.max(0.0, Math.min(1.0, diffy2Position));

        // Update servo positions
        diffy1.setPosition(diffy1Position);
        diffy2.setPosition(diffy2Position);

        // Update previous state variables
        prevLeftBumper = gamepad1.left_bumper;
        prevRightBumper = gamepad1.right_bumper;
        prevLeftTriggerPressed = leftTriggerPressed;
        prevRightTriggerPressed = rightTriggerPressed;

        // Telemetry for debugging
        telemetry.addData("Diffy1 Position", "%.2f", diffy1Position);
        telemetry.addData("Diffy2 Position", "%.2f", diffy2Position);
        telemetry.update();
    }
}