package org.firstinspires.ftc.teamcode.config.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.config.util.HWValues;
import org.firstinspires.ftc.teamcode.config.util.action.RunAction;

import org.firstinspires.ftc.teamcode.config.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.config.pedroPathing.util.CustomPIDFCoefficients;
import org.firstinspires.ftc.teamcode.config.pedroPathing.util.FeedForwardConstant;
import org.firstinspires.ftc.teamcode.config.pedroPathing.util.PIDFController;

public class Arm {

    private Telemetry telemetry;
    public DcMotorEx arm;
    private int pos;
    public RunAction toZero, toCollect, toClear, toScore, toAttach, toWinch;

    public PIDController armPID;

    public static double target;
    public static double TARGET_MAX = -500;
    public static  double TARGET_MIN = -4500;
    public static double kP = 0.1, kI = 0, kD = 0.0002;
    public static double f = 0.00;
    public static double ARM_SPEED = 0.05;

    // TODO
    private final double ARM_TICKS_PER_DEGREE = 19.7924893140647;

    public Arm(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        arm = hardwareMap.get(DcMotorEx.class, HWValues.ARM);
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armPID = new PIDController(kP, kI, kD);
        toZero = new RunAction(this::toZero);
        toCollect = new RunAction(this::toCollect);
        toClear = new RunAction(this::toClear);
        toScore = new RunAction(this::toScore);
        toAttach = new RunAction(this::toAttach);
        toWinch = new RunAction(this::toWinch);
    }

    public void manual(double n) { setTarget(target + n * ARM_SPEED);}
    public void setTarget(double b) {
        if (b > TARGET_MAX) {
            target = TARGET_MAX;
        } else if (b < TARGET_MIN) {
            target = TARGET_MIN;
        } else {
            target = b;
        }
    }
    public double getTarget() {return target;}

    // PID Stuff //
    public void updatePIDF() {
        armPID.setPID(kP, kI, kD);
        updatePos();
        double pid = armPID.calculate(pos, target);
        double ff = Math.sin(Math.toRadians(target/ ARM_TICKS_PER_DEGREE)) * f;
        double power = pid + ff;

        arm.setPower(power);
        telemetry.addData("arm pos", pos);
        telemetry.addData("arm target", target);
    }

    // Positions //

    public void toZero() {
        updatePos();
        setTarget(0);
    }
    public void toCollect() {
        updatePos();
        setTarget(250 * ARM_TICKS_PER_DEGREE);
    }

    public void toClear() {
        updatePos();
        setTarget(230 * ARM_TICKS_PER_DEGREE);
    }

    public void toScore() {
        updatePos();
        setTarget(160 * ARM_TICKS_PER_DEGREE);
    }

    public void toAttach() {
        updatePos();
        setTarget(120 * ARM_TICKS_PER_DEGREE);
    }

    public void toWinch() {
        updatePos();
        setTarget(15 * ARM_TICKS_PER_DEGREE);
    }

    // Util //

    public void resetEncoder() {
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public double getPos() {return pos;}

    public void updatePos() {
        pos = arm.getCurrentPosition();
    }

    // Init + Start //

    public void init() {
        resetEncoder();
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        pos = arm.getCurrentPosition();
    }

    public void start() {}
}
