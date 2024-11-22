package org.firstinspires.ftc.teamcode.config.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.AnalogInput;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.config.pedroPathing.util.CustomPIDFCoefficients;
import org.firstinspires.ftc.teamcode.config.pedroPathing.util.FeedForwardConstant;
import org.firstinspires.ftc.teamcode.config.pedroPathing.util.PIDFController;

import static org.firstinspires.ftc.teamcode.config.util.RobotConstants.*;

import org.firstinspires.ftc.teamcode.config.util.action.RunAction;

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
public class Arm {
    private PIDController controller;
    public static double p =0.05, i = 0, d = 0.002;
    public static double f = 0;

    public static double target = 0;

    public static int offset = -217;

    public DcMotorEx arm_motor;
    private AnalogInput encoder;

    public RunAction  autoArmIntake2 , armLowBasket,armIntake,armSpecimen, armObservation,armObservationUp, armMax, armSpecimenScore, armClear, armUpright, autoArmPreSpecimen, autoArmSpecimenDown, autoArmSpecimen2, armWallIntakeFinal;

    public Arm(HardwareMap hardwareMap) {
        controller = new PIDController(p, i, d);

        arm_motor = hardwareMap.get(DcMotorEx.class, "arm");
        arm_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        encoder = hardwareMap.get(AnalogInput.class, "enc");

        target = getCurrentPosition();

        armLowBasket = new RunAction(this::armLowBasket);
        armIntake = new RunAction(this::armIntake);
        armSpecimen = new RunAction(this::armSpecimen);
        armObservation = new RunAction(this::armObservation);
        armObservationUp = new RunAction(this::armObservationUp);
        armMax = new RunAction(this::armMax);
        armSpecimenScore = new RunAction(this::armSpecimenScore);
        armClear = new RunAction(this::armClear);
        armUpright = new RunAction(this::armUpright);
        autoArmPreSpecimen = new RunAction(this::autoArmPreSpecimen);
        autoArmSpecimenDown = new RunAction(this::autoArmSpecimen);
        autoArmSpecimen2 = new RunAction(this::autoArmSpecimen2);
        armWallIntakeFinal = new RunAction(this::armWallIntakeFinal);
        autoArmIntake2 = new RunAction(this::autoArmIntake2);
    }

    public void update() {
        controller.setPID(p, i, d);
        double armPos = getCurrentPosition();
        double pid = controller.calculate(armPos, target);
        double ff = Math.sin(Math.toRadians(target)) * f;

        double power = pid + ff;

        arm_motor.setPower(power);
    }

    public double getCurrentPosition() {
        return (encoder.getVoltage() / 3.2 * 360 + offset) % 360;
    }

    public void setArmTarget(double new_target, double direction) {
        target = new_target;
    }
    public void setArmTarget(double new_target) {
        target = new_target;
    }
    public double getArmCurrent() {return arm_motor.getCurrent(CurrentUnit.AMPS);}

    public double getArmTarget() {
        return target;
    }

    public void manual(double n) {
        setArmTarget(target + n * ARM_SPEED);
    }


    public void armLowBasket() {setArmTarget(ARM_LOWBASKET);}
    public void armIntake() {setArmTarget(ARM_INTAKE);}
    public void armSpecimen() {
        setArmTarget(ARM_SPECIMEN);
    }
    public void armObservation() {setArmTarget(ARM_OBSERVATION);}
    public void armWallIntakeFinal() {setArmTarget(90);}
    public void armObservationUp() {setArmTarget(0.9);}
    public void armMax() {setArmTarget(ARM_MAX);}
    public void armSpecimenScore() {setArmTarget(ARM_SPECIMEN_SCORE);}

    public void armClear() {setArmTarget(ARM_CLEAR);}
    public void armUpright() {setArmTarget(-5);}
    public void autoArmPreSpecimen() {setArmTarget(55);}
    public void autoArmSpecimen() {setArmTarget(65);}
    public void autoArmSpecimen2() {setArmTarget(55);}
   public void autoArmIntake2(){setArmTarget(-90);}





}