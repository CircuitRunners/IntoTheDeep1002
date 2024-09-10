package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.teamcode.util.HWValues;

import org.firstinspires.ftc.teamcode.subsystems.Arm;

@TeleOp(name = "Main TeleOp")
public class MainTeleop extends OpMode {
    MecanumDrive drive = new MecanumDrive();
    SparkFunOTOS sparkfunOTOS;

    Arm arm;

    @Override
    public void init() {
        drive.init(hardwareMap);
        sparkfunOTOS = hardwareMap.get(SparkFunOTOS.class, HWValues.OTOS);
        arm = new Arm(hardwareMap);
        configureOTOS();
    }

    @Override
    public void loop() {
        SparkFunOTOS.Pose2D pos = sparkfunOTOS.getPosition();
        telemetry.addData("X (inch)", pos. x);
        telemetry.addData("Y (inch)", pos.y);
        telemetry.addData("Heading (degrees)", pos.h);
        double forward = -gamepad1.left_stick_y;
        double right = gamepad1.left_stick_x;
        double rotate = gamepad1.right_stick_x;

        // Reset the tracking if the user requests it
        if (gamepad1.y) {
            sparkfunOTOS.resetTracking();
        }

        // Re-calibrate the IMU if the user requests it
        if (gamepad1.x) {
            sparkfunOTOS.calibrateImu();
        }

        if (gamepad1.b) {
            drive.holdPosition();
        }

        // Inform user of available controls
        telemetry.addLine("Press Y (triangle) on Gamepad to reset tracking");
        telemetry.addLine("Press X (square) on Gamepad to calibrate the IMU");

        telemetry.addLine();

        driveFieldRelative(forward, right, rotate);
        arm.armUpdate();
    }

    private void configureOTOS() {
        sparkfunOTOS.setLinearUnit(DistanceUnit.INCH);
        sparkfunOTOS.setAngularUnit(AngleUnit.DEGREES);
        sparkfunOTOS.setOffset(new SparkFunOTOS.Pose2D(0, Constants.Y_OFFSET, 0));
        sparkfunOTOS.setLinearScalar(Constants.L_SCALER);
        sparkfunOTOS.setAngularScalar(Constants.A_SCALER);
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
