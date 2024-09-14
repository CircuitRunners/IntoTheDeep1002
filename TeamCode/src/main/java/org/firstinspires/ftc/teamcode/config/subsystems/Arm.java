package org.firstinspires.ftc.teamcode.config.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.firstinspires.ftc.teamcode.config.util.RobotConstants;
import org.firstinspires.ftc.teamcode.config.util.HWValues;
import org.firstinspires.ftc.teamcode.config.util.action.RunAction;

public class Arm {

    private Telemetry telemetry;
    public DcMotorEx arm;
    private int pos;
    public RunAction toZero, toCollect, toLowBucket, toLowChamber, toHighChamber;

    public PIDController armPID;

    public static int target;
    public static double p = 0, i = 0, d = 0;
    public static double f = 0.01;
    // TODO
    private final double ticks_in_degrees = 1.5;

    public Arm(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        arm = hardwareMap.get(DcMotorEx.class, HWValues.ARM);
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        armPID = new PIDController(p, i, d);
        toZero = new RunAction(this::toZero);
        toCollect = new RunAction(this::toCollect);
        toLowBucket = new RunAction(this::toLowBucket);
        toLowChamber = new RunAction(this::toLowChamber);
        toHighChamber = new RunAction(this::toHighChamber);


    }

    public void manual(double n) { arm.setPower(n); }
    public void setTarget(int b) { target = b; }


    public void updatePIDF() {
        armPID.setPID(p, i, d);
        updatePos();
        double pid = armPID.calculate(pos, target);
        double ff = Math.cos(Math.toRadians(target/ticks_in_degrees)) * f;
        double power = pid + ff;

        arm.setPower(power);
        telemetry.addData("arm pos", pos);
        telemetry.addData("arm target", target);
    }

    public void toZero() {
        updatePos();
        setTarget(0);
    }
    public void toCollect() {
        updatePos();
        setTarget(0);
    }
    public void toLowBucket() {
        updatePos();
        setTarget(0);
    }

    public void toLowChamber() {
        updatePos();
        setTarget(0);
    }

    public void toHighChamber() {
        updatePos();
        setTarget(0);
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
