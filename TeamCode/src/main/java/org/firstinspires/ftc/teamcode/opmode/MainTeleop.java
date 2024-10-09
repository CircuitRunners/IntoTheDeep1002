package org.firstinspires.ftc.teamcode.opmode;


import org.firstinspires.ftc.teamcode.config.subsystems.Arm;
import org.firstinspires.ftc.teamcode.config.subsystems.EndEffector;
import org.firstinspires.ftc.teamcode.config.subsystems.MecanumDrive;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import org.firstinspires.ftc.teamcode.config.util.RobotConstants;
import org.firstinspires.ftc.teamcode.config.util.HWValues;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import static org.firstinspires.ftc.teamcode.config.util.RobotConstants.*;


@TeleOp(name = "MainTeleop", group = "Test")
public class MainTeleop extends OpMode {
    MecanumDrive drive = new MecanumDrive();
    SparkFunOTOS sparkfunOTOS;
    private Arm arm;
    private EndEffector endEffector;

    private boolean prevLeftBumper = false;
    private boolean prevRightBumper = false;
    private boolean prevLeftTriggerPressed = false;
    private boolean prevRightTriggerPressed = false;
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
        if (gamepad1.y) {
            sparkfunOTOS.resetTracking();
        }
        if (gamepad1.x) {
            sparkfunOTOS.calibrateImu();
        }
        driveFieldRelative(forward, right, rotate);

        if (gamepad2.dpad_down) {
            endEffector.openClaw();
            arm.setArmTarget(ARM_INTAKE);
            endEffector.intakePositionH();
        }
        if (arm.getArmTarget() < -4) {
            if (gamepad2.triangle) {
                endEffector.intakePositionH();
            }
            if (gamepad2.cross) {
                endEffector.intakePositionV();
            }
            if (gamepad2.square) {
                endEffector.intakePositionAL();
            }
            if (gamepad2.circle) {
                endEffector.intakePositionAR();
            }
        }

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
        if (gamepad2.right_stick_button) {
            arm.setArmTarget(ARM_MAX);
            // arm.armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            endEffector.openClaw();
            endEffector.idlePosition();
        }

        // Edge detection for left bumper (decrement diffy1)
        if (gamepad2.left_bumper && !prevLeftBumper) {
            endEffector.diffy1decrement();
        }

        // Edge detection for right bumper (increment diffy1)
        if (gamepad2.right_bumper && !prevRightBumper) {
            endEffector.diffy1increment();
        }

        // Determine if triggers are pressed (threshold > 0.5)
        boolean leftTriggerPressed = gamepad2.left_trigger > 0.5;
        boolean rightTriggerPressed = gamepad2.right_trigger > 0.5;

        // Edge detection for left trigger (decrement diffy2)
        if (leftTriggerPressed && !prevLeftTriggerPressed) {
            endEffector.diffy2decrement();
        }

        // Edge detection for right trigger (increment diffy2)
        if (rightTriggerPressed && !prevRightTriggerPressed) {
            endEffector.diffy2increment();
        }


        // Update previous state variables
        prevLeftBumper = gamepad2.left_bumper;
        prevRightBumper = gamepad2.right_bumper;
        prevLeftTriggerPressed = leftTriggerPressed;
        prevRightTriggerPressed = rightTriggerPressed;


        if (gamepad2.start) {
            endEffector.openClaw();
        }
        if (gamepad2.back) {
            endEffector.closeClaw();
        }

        arm.manual(gamepad2.right_stick_y);


        // Telemetry
        telemetry.addData("Arm Target", arm.getArmTarget());
        telemetry.addData("Arm Pos", arm.armMotor.getCurrentPosition());
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

}