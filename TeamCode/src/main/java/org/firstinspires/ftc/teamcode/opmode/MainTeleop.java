package org.firstinspires.ftc.teamcode.opmode;


import org.firstinspires.ftc.teamcode.config.subsystems.Arm;
import org.firstinspires.ftc.teamcode.config.subsystems.EndEffector;
import org.firstinspires.ftc.teamcode.config.subsystems.MecanumDrive;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import org.firstinspires.ftc.teamcode.config.util.RobotConstants;
import org.firstinspires.ftc.teamcode.config.util.HWValues;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.config.util.action.Action;
import org.firstinspires.ftc.teamcode.config.util.action.SequentialAction;

import org.firstinspires.ftc.teamcode.config.util.action.Action;
import org.firstinspires.ftc.teamcode.config.util.action.Actions;
import org.firstinspires.ftc.teamcode.config.util.action.ParallelAction;
import org.firstinspires.ftc.teamcode.config.util.action.RunAction;
import org.firstinspires.ftc.teamcode.config.util.action.SequentialAction;
import org.firstinspires.ftc.teamcode.config.util.action.SleepAction;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import static org.firstinspires.ftc.teamcode.config.util.RobotConstants.*;


@TeleOp(name = "MainTeleop", group = "Test")
public class MainTeleop extends OpMode {
    MecanumDrive drive = new MecanumDrive();
    SparkFunOTOS sparkfunOTOS;
    private Arm arm;
    private EndEffector endEffector;

    private boolean shouldReverseInput = false; // Determines whether the input should be reversed
    private boolean joystickWasZero = true; // Tracks if the joystick was released
    private boolean prevTrianglePressed = false;

    private boolean prevDown = false;
    private int intakeState = 0;
    @Override
    public void init() {
        telemetry.addLine("Initializing...");
        telemetry.update();

        arm = new Arm(hardwareMap);
        endEffector = new EndEffector(hardwareMap);
        drive.init(hardwareMap);
        sparkfunOTOS = hardwareMap.get(SparkFunOTOS.class, HWValues.OTOS);
        configureOTOS();
        telemetry.addLine("Ready!");
        telemetry.update();
    }

    /**
     * This runs the OpMode. This is only drive control with Pedro Pathing live centripetal force
     * correction.
     */
    @Override
    public void loop() {
        SparkFunOTOS.Pose2D pos = sparkfunOTOS.getPosition();
        double forward = -gamepad1.left_stick_y;
        double right = gamepad1.left_stick_x;
        double rotate = gamepad1.right_stick_x;
        if (gamepad1.x) {
            sparkfunOTOS.resetTracking();
        }
        if (gamepad1.y) {
            sparkfunOTOS.calibrateImu();
        }
        driveFieldRelative(forward, right, rotate);

        if (gamepad2.right_stick_y > 0.5) {
            endEffector.diffy1increment();
            endEffector.diffy2increment();
        }
        if (gamepad2.right_stick_y < -0.5) {
            endEffector.diffy1decrement();
            endEffector.diffy2decrement();
        }
        if (gamepad2.right_stick_x > 0.5) {
            endEffector.diffy1increment();
            endEffector.diffy2decrement();
        }
        if (gamepad2.right_stick_x < -0.5) {
            endEffector.diffy1decrement();
            endEffector.diffy2increment();
        }

        if (gamepad2.dpad_down && !prevDown) {
            if (arm.getArmTarget() != ARM_CLEAR && intakeState == 0) {
                endEffector.openClaw();
                arm.setArmTarget(ARM_CLEAR);
                endEffector.intakeClear();
                intakeState = 1;
            } else if (intakeState == 0) {
                endEffector.openClaw();
                endEffector.intakeClear();
                intakeState = 1;
            } else if (intakeState == 1) {
                endEffector.openClaw();
                arm.setArmTarget(ARM_INTAKE);
                intakeState = 2;
            } else if (intakeState == 2) {
                Actions.runBlocking(intakePickup());
                intakeState = 0;
            }
        }
        prevDown = gamepad2.dpad_down;
        if (gamepad2.dpad_up) {
            arm.setArmTarget(ARM_LOWBASKET);
            endEffector.basketPosition();
        }

        if (gamepad2.dpad_right) {
            if (arm.getArmTarget() != ARM_SPECIMEN) {
                arm.setArmTarget(ARM_SPECIMEN);
                endEffector.specimenPosition();
            } else {
                arm.setArmTarget(ARM_SPECIMEN_SCORE);
                endEffector.specimenScorePosition();
            }

        }
        if (gamepad2.dpad_left) {
            arm.setArmTarget(ARM_MAX);
            endEffector.obsPosition();
        }
        if (gamepad2.left_stick_button) {
            arm.setArmTarget(ARM_MAX);
            endEffector.openClaw();
            endEffector.idlePosition();
        }

        if (gamepad2.right_bumper && arm.getArmTarget() == ARM_MAX) {
            arm.resetEncoder();
        }

        if (gamepad2.triangle && !prevTrianglePressed) {
            endEffector.switchClaw();
        }
        prevTrianglePressed = gamepad2.triangle;

       controlArm(gamepad2.left_stick_y);

        // Telemetry
        telemetry.addData("Arm Target", arm.getArmTarget());
        telemetry.addData("Arm Pos", arm.armAngle());
        telemetry.addData("Claw Position", endEffector.getClawPosition());
        telemetry.addData("Diffy1 Position", "%.2f", endEffector.getDiffy1Position());
        telemetry.addData("Diffy2 Position", "%.2f", endEffector.getDiffy2Position());
        telemetry.update();
        telemetry.update();
        arm.update();
    }
    private void configureOTOS() {
        sparkfunOTOS.setLinearUnit(DistanceUnit.INCH);
        sparkfunOTOS.setAngularUnit(AngleUnit.DEGREES);
        sparkfunOTOS.setOffset(new SparkFunOTOS.Pose2D(0, RobotConstants.Y_OFFSET, 0));
        sparkfunOTOS.setLinearScalar(RobotConstants.L_SCALER);
        sparkfunOTOS.setAngularScalar(RobotConstants.A_SCALER);
        sparkfunOTOS.resetTracking();
        sparkfunOTOS.setPosition(new SparkFunOTOS.Pose2D(0, 0, 0));
        sparkfunOTOS.calibrateImu(255, false);
    }
    private void driveFieldRelative(double forward, double right, double rotate) {
        SparkFunOTOS.Pose2D pos = sparkfunOTOS.getPosition();
        double robotAngle = Math.toRadians(pos.h);
        double theta = Math.atan2(forward, right);
        double r = Math.hypot(forward, right);
        theta = AngleUnit.normalizeRadians(theta - robotAngle);

        // convert back to cartesian
        double newForward = r * Math.sin(theta);
        double newRight = r * Math.cos(theta);

        drive.drive(newForward, newRight, rotate);
    }

    public void controlArm(double joystickInput) {
        // Get the current arm angle
        double target = arm.getArmTarget();

        boolean isAboveThreshold = target > -1.63;

        if (Math.abs(joystickInput) < 0.05) {
            joystickWasZero = true; // Joystick is considered released
        } else {
            if (joystickWasZero) {
                shouldReverseInput = isAboveThreshold;
            }
            joystickWasZero = false; // Joystick is now active
        }

        // Adjust the joystick input based on the shouldReverseInput flag
        double adjustedInput = shouldReverseInput ? -joystickInput : joystickInput;

        // Move the arm using the adjusted joystick input
        arm.manual(-adjustedInput);
    }

    public Action intakePickup() {
        return new SequentialAction(
                endEffector.closeClaw,
                new SleepAction(0.4),
                arm.armClear
        );
    }

}