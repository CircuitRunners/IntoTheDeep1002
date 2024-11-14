package org.firstinspires.ftc.teamcode.opmode.Tuners;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.AnalogInput;
import org.firstinspires.ftc.teamcode.config.pedroPathing.util.CustomPIDFCoefficients;
import org.firstinspires.ftc.teamcode.config.pedroPathing.util.FeedForwardConstant;
import org.firstinspires.ftc.teamcode.config.pedroPathing.util.PIDFController;


@TeleOp
@Config
public class ArmTuner extends OpMode {

    public static double kP = 0;
    public static double kI = 0;
    public static double kD = 0;
    public static double f = 0.05;

    public static double target = 0;
    public static double offset = -218;
    public CustomPIDFCoefficients PID = new CustomPIDFCoefficients(kP, kI, kD, new ArmTuner.ArmPIDF());

    class ArmPIDF implements FeedForwardConstant {
        @Override
        public double getConstant(double input) {
            return Math.sin(input) * f;
        }
    }

    public DcMotorEx armMotor;
    public AnalogInput enc;

    public double armAngle() {
        return (enc.getVoltage() / 3.2 * 360 + offset) % 360;
    }

    private PIDFController armPID = new PIDFController(PID);



    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        armMotor = hardwareMap.get(DcMotorEx.class, "arm");
        enc = hardwareMap.get(AnalogInput.class, "enc");
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        target = armAngle();
    }

    @Override
    public void loop() {
        armPID.updateFeedForwardInput(armAngle());
        armPID.setTargetPosition(target);
        armPID.updatePosition(armAngle());
        armMotor.setPower(armPID.runPIDF());

        target += gamepad1.left_stick_y;
        kD += (gamepad1.right_stick_y / 0.1);


        telemetry.addData("pos", armAngle());
        telemetry.addData("target", target);
        telemetry.addData("power", armMotor.getPower());
        telemetry.addData("P", kP);
        telemetry.addData("I", kI);
        telemetry.addData("D", kD);
        telemetry.update();
    }
}
