package org.firstinspires.ftc.teamcode.teleop.tuners;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Const;
import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.teamcode.util.Constants;

@TeleOp
public class ArmTuner extends OpMode {
    private PIDController controller;
    private final double ticks_in_degree = Constants.ARM_TICKS_PER_DEGREE;
    private DcMotorEx arm_motor;

    @Override
    public void init() {
        controller = new PIDController(Constants.ARM_P, Constants.ARM_I, Constants.ARM_D);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        arm_motor = hardwareMap.get(DcMotorEx.class, "arm_motor0");
    }

    @Override
    public void loop() {
        controller.setPID(Constants.ARM_P, Constants.ARM_I, Constants.ARM_D);
        int armPos = arm_motor.getCurrentPosition();
        double pid = controller.calculate(armPos, Constants.ARM_TEST_TARGET);
        double ff = Math.cos(Math.toRadians(Constants.ARM_TEST_TARGET / ticks_in_degree)) * Constants.ARM_F;

        double power = pid + ff;

        arm_motor.setPower(power);

        telemetry.addData("pos", armPos);
        telemetry.addData("target", Constants.ARM_TEST_TARGET);
        telemetry.update();
    }
}
