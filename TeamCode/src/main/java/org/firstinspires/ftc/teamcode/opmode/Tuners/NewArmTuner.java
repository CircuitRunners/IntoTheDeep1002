package org.firstinspires.ftc.teamcode.opmode.Tuners;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.AnalogInput;

@Config
@TeleOp
public class NewArmTuner extends OpMode {
    private PIDController controller;
    public static double p =0.05, i = 0, d = 0;
    public static double f = 0;

    public static int target = 0;

    public static int offset = -217;

    private DcMotorEx arm_motor;
    private AnalogInput encoder;


    @Override
    public void init() {
        controller = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        arm_motor = hardwareMap.get(DcMotorEx.class, "arm");
        arm_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        encoder = hardwareMap.get(AnalogInput.class, "enc");
    }

    @Override
    public void loop() {
        controller.setPID(p, i, d);
        double armPos = getCurrentPosition();
        double pid = controller.calculate(armPos, target);
        double ff = Math.sin(Math.toRadians(target)) * f;

        double power = pid + ff;

        arm_motor.setPower(power);

        telemetry.addData("pos ", armPos);
        telemetry.addData("target ", target);
        telemetry.update();


    }

    private double getCurrentPosition() {
        return (encoder.getVoltage() / 3.2 * 360 + offset) % 360;
    }
}
