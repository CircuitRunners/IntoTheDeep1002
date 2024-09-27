package org.firstinspires.ftc.teamcode.opmode.Tuners;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.config.util.RobotConstants;

@TeleOp
public class ArmTuner extends OpMode {
    private PIDController controller;
    public static double kP = 0.1, kI = 0, kD = 0.0002;
    public static double f = 0.00;
    public static int target = 0;

    private final double ticks_in_degree = 19.7924893140647;
    private DcMotorEx arm_motor;



    @Override
    public void init() {
        controller = new PIDController(kP, kI, kD);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        arm_motor = hardwareMap.get(DcMotorEx.class, "arm");
    }

    @Override
    public void loop() {
        controller.setPID(kP, kI, kD);
        int armPos = arm_motor.getCurrentPosition();
        double pid = controller.calculate(armPos, target);
        double ff = Math.sin(Math.toRadians(target / ticks_in_degree)) * f;

        double power = pid + ff;

        arm_motor.setPower(power);

        telemetry.addData("pos", armPos);
        telemetry.addData("target", target);
        telemetry.addData("power", arm_motor.getPower());
        telemetry.update();
    }
}
