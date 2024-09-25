package org.firstinspires.ftc.teamcode.config.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.config.pedroPathing.util.CustomPIDFCoefficients;
import org.firstinspires.ftc.teamcode.config.pedroPathing.util.FeedForwardConstant;
import org.firstinspires.ftc.teamcode.config.pedroPathing.util.PIDFController;
import org.firstinspires.ftc.teamcode.opmode.ManualDrive;

@Config
public class Arm {
    public static double kP = 1;
    public static double kI = 0.02;
    public static double kD = 0.02;
    public static double f = -0.1;
    public static double TICK_PER_RAD = ((((1+(46.0/17.0))) * (1+(46.0/17.0))) * (1+(46.0/17.0)) * 28) / 2*Math.PI / 3.2;
    public static CustomPIDFCoefficients PID = new CustomPIDFCoefficients(kP, kI, kD, new Arm.ArmPIDF());

    private double armStart = 0.0;
    public static double ARM_TARGET = 0.0;
    public static double ARM_OFF = -2.01;
    public static double ARM_SPEED = 0.05;
    public static double ARM_MIN = -4.85;
    public static double ARM_MAX = 1.95;
    public static double ARM_LOWBASKET = -0.91;
    public static double ARM_INTAKE = -4.2655;
    public static double ARM_SPECIMEN = -2.5;
    public static double OBSERVATION = 1;
    public static double HANG = 1;

    static class ArmPIDF implements FeedForwardConstant {
        @Override
        public double getConstant(double input) {
            return Math.sin(input) * f;
        }
    }

    private DcMotorEx armMotor;

    private double armAngle() {
        return (armMotor.getCurrentPosition() - armStart)/TICK_PER_RAD - ARM_OFF;
    }

    private PIDFController armPID = new PIDFController(PID);

    public Arm(HardwareMap hardwareMap) {
        armMotor = hardwareMap.get(DcMotorEx.class, "arm");
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armStart = armMotor.getCurrentPosition();
        ARM_TARGET = armAngle();
    }

    public void update() {
        armPID.updateFeedForwardInput(armAngle());
        armPID.setTargetPosition(ARM_TARGET);
        armPID.updatePosition(armAngle());
        armMotor.setPower(armPID.runPIDF());
    }

    public void setArmTarget(double target) {
        ARM_TARGET = target;
        if (ARM_TARGET < ARM_MIN) {
            ARM_TARGET = ARM_MIN;
        } else if (ARM_TARGET > ARM_MAX) {
            ARM_TARGET = ARM_MAX;
        }
    }

    public double getArmTarget() {
        return ARM_TARGET;
    }

    public void manual(double n) {
        setArmTarget(ARM_TARGET + n * ARM_SPEED);
    }

    public void resetEncoder() {
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

}



//package org.firstinspires.ftc.teamcode.config.subsystems;
//
//import com.acmerobotics.dashboard.config.Config;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//
//import org.firstinspires.ftc.teamcode.config.pedroPathing.util.CustomPIDFCoefficients;
//import org.firstinspires.ftc.teamcode.config.pedroPathing.util.FeedForwardConstant;
//import org.firstinspires.ftc.teamcode.config.pedroPathing.util.PIDFController;
//
//@Config
//public class Arm {
//    public static double ARM_F = -0.1;
//    public static double TICK_PER_RAD = ((((1 + (46.0 / 17.0))) * (1 + (46.0 / 17.0))) * (1 + (46.0 / 17.0)) * 28) / (2 * Math.PI) / 3.2;
//    // CHANGE THIS: The "/ 3.2" at the end is the gear ratio; adjust it for your robot.
//
//    public static double ARM_OFF = -2.01;
//    public static CustomPIDFCoefficients PID = new CustomPIDFCoefficients(1, 0.02, 0.02, new ArmPIDF());
//    public static double ARM_SPEED = 0.05;
//
//    public static double ARM_MIN = -2.25;
//    public static double ARM_MAX = 1.9;
//
//    private DcMotorEx armMotor;
//    private PIDFController armPID;
//    private double armStart = 0.0;
//    private double armTarget = 0.0;
//    private double lastArmPower = 0.0;
//
//    public Arm(HardwareMap hardwareMap) {
//        armMotor = hardwareMap.get(DcMotorEx.class, "arm");
//        armStart = armMotor.getCurrentPosition();
//        armPID = new PIDFController(PID);
//        armTarget = getArmAngle();
//    }
//
//    public void update() {
//        armPID.updateFeedForwardInput(getArmAngle());
//        armPID.setTargetPosition(armTarget);
//        armPID.updatePosition(getArmAngle());
//        lastArmPower = armPID.runPIDF();
//        armMotor.setPower(lastArmPower);
//    }
//
//    public void setArmTarget(double target) {
//        armTarget = target;
//        if (armTarget < ARM_MIN) {
//            armTarget = ARM_MIN;
//        } else if (armTarget > ARM_MAX) {
//            armTarget = ARM_MAX;
//        }
//    }
//
//    public double getArmTarget() {
//        return armTarget;
//    }
//
//    public double getArmAngle() {
//        return (armMotor.getCurrentPosition() - armStart) / TICK_PER_RAD - ARM_OFF;
//    }
//
//    public void adjustArmTarget(double delta) {
//        setArmTarget(armTarget + delta);
//    }
//
//    public double getArmPower() {
//        return lastArmPower;
//    }
//
//    static class ArmPIDF implements FeedForwardConstant {
//        @Override
//        public double getConstant(double input) {
//            return Math.sin(input) * ARM_F;
//        }
//    }
//}
//
////package org.firstinspires.ftc.teamcode.config.subsystems;
////
////import com.acmerobotics.dashboard.FtcDashboard;
////import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
////import com.arcrobotics.ftclib.controller.PIDController;
////import com.qualcomm.robotcore.hardware.DcMotor;
////import com.qualcomm.robotcore.hardware.DcMotorEx;
////import com.qualcomm.robotcore.hardware.HardwareMap;
////import org.firstinspires.ftc.robotcore.external.Telemetry;
////
////import org.firstinspires.ftc.teamcode.R;
////import org.firstinspires.ftc.teamcode.config.util.HWValues;
////import org.firstinspires.ftc.teamcode.config.util.action.RunAction;
////
////import org.firstinspires.ftc.teamcode.config.pedroPathing.follower.Follower;
////import org.firstinspires.ftc.teamcode.config.pedroPathing.util.CustomPIDFCoefficients;
////import org.firstinspires.ftc.teamcode.config.pedroPathing.util.FeedForwardConstant;
////import org.firstinspires.ftc.teamcode.config.pedroPathing.util.PIDFController;
////
////public class Arm {
////
////    private Telemetry telemetry;
////    public DcMotorEx arm;
////    private int pos;
////    public RunAction toZero, toCollect, toClear, toScore, toAttach, toWinch;
////
////    public PIDController armPID;
////
////    public static double target;
////    public static double TARGET_MAX = -500;
////    public static  double TARGET_MIN = -4500;
////    public static double kP = 0.1, kI = 0, kD = 0.0002;
////    public static double f = 0.00;
////    public static double ARM_SPEED = 0.05;
////
////    // TODO
////    private final double ARM_TICKS_PER_DEGREE = 19.7924893140647;
////
////    public Arm(HardwareMap hardwareMap, Telemetry telemetry) {
////        this.telemetry = telemetry;
////        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
////        arm = hardwareMap.get(DcMotorEx.class, HWValues.ARM);
////        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
////        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
////        armPID = new PIDController(kP, kI, kD);
////        toZero = new RunAction(this::toZero);
////        toCollect = new RunAction(this::toCollect);
////        toClear = new RunAction(this::toClear);
////        toScore = new RunAction(this::toScore);
////        toAttach = new RunAction(this::toAttach);
////        toWinch = new RunAction(this::toWinch);
////    }
////
////    public void manual(double n) { setTarget(target + n * ARM_SPEED);}
////    public void setTarget(double b) {
////        if (b > TARGET_MAX) {
////            target = TARGET_MAX;
////        } else if (b < TARGET_MIN) {
////            target = TARGET_MIN;
////        } else {
////            target = b;
////        }
////    }
////    public double getTarget() {return target;}
////
////    // PID Stuff //
////    public void updatePIDF() {
////        armPID.setPID(kP, kI, kD);
////        updatePos();
////        double pid = armPID.calculate(pos, target);
////        double ff = Math.sin(Math.toRadians(target/ ARM_TICKS_PER_DEGREE)) * f;
////        double power = pid + ff;
////
////        arm.setPower(power);
////        telemetry.addData("arm pos", pos);
////        telemetry.addData("arm target", target);
////    }
////
////    // Positions //
////
////    public void toZero() {
////        updatePos();
////        setTarget(0);
////    }
////    public void toCollect() {
////        updatePos();
////        setTarget(250 * ARM_TICKS_PER_DEGREE);
////    }
////
////    public void toClear() {
////        updatePos();
////        setTarget(230 * ARM_TICKS_PER_DEGREE);
////    }
////
////    public void toScore() {
////        updatePos();
////        setTarget(-3000);
////    }
////
////    public void toAttach() {
////        updatePos();
////        setTarget(120 * ARM_TICKS_PER_DEGREE);
////    }
////
////    public void toWinch() {
////        updatePos();
////        setTarget(15 * ARM_TICKS_PER_DEGREE);
////    }
////
////    // Util //
////
////    public void resetEncoder() {
////        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
////    }
////
////    public double getPos() {return pos;}
////
////    public void updatePos() {
////        pos = arm.getCurrentPosition();
////    }
////
////    // Init + Start //
////
////    public void init() {
////        resetEncoder();
////        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
////        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
////        pos = arm.getCurrentPosition();
////    }
////
////    public void start() {}
////}
