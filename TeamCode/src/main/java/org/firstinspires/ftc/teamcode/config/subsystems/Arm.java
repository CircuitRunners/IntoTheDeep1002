package org.firstinspires.ftc.teamcode.config.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.config.pedroPathing.util.CustomPIDFCoefficients;
import org.firstinspires.ftc.teamcode.config.pedroPathing.util.FeedForwardConstant;
import org.firstinspires.ftc.teamcode.config.pedroPathing.util.PIDFController;

import static org.firstinspires.ftc.teamcode.config.util.RobotConstants.*;

import org.firstinspires.ftc.teamcode.config.util.action.RunAction;

public class Arm {
    public static CustomPIDFCoefficients PID = new CustomPIDFCoefficients(kP, kI, kD, new Arm.ArmPIDF());

    static class ArmPIDF implements FeedForwardConstant {
        @Override
        public double getConstant(double input) {
            return Math.sin(input) * f;
        }
    }

    public DcMotorEx armMotor;

    public RunAction armLowBasket, armIntake, armSpecimen, armObservation, armMax, armSpecimenScore, armClear;

    public double armAngle() {
        return (armMotor.getCurrentPosition() - armStart)/TICK_PER_RAD - ARM_OFF;
    }

    private PIDFController armPID = new PIDFController(PID);

    public Arm(HardwareMap hardwareMap) {
        armMotor = hardwareMap.get(DcMotorEx.class, "arm");
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armStart = armMotor.getCurrentPosition();
        ARM_TARGET = armAngle();

        armLowBasket = new RunAction(this::armLowBasket);
        armIntake = new RunAction(this::armIntake);
        armSpecimen = new RunAction(this::armSpecimen);
        armObservation = new RunAction(this::armObservation);
        armMax = new RunAction(this::armMax);
        armSpecimenScore = new RunAction(this::armSpecimenScore);
        armClear = new RunAction(this::armClear);
    }

    public void update() {
        armPID.updateFeedForwardInput(armAngle());
        armPID.setTargetPosition(ARM_TARGET);
        armPID.updatePosition(armAngle());
        armMotor.setPower(armPID.runPIDF());
    }

    public void setArmTarget(double target) {
        double currentDraw = armMotor.getCurrent(CurrentUnit.AMPS);

        // If current exceeds the threshold, don't change the target to prevent damage
        if (currentDraw > 6) {
            // Optionally, log or alert to indicate that the current threshold was reached
            System.out.println("Current threshold exceeded! Target position not updated.");
            return; // Exit the method to prevent setting a new target
        }
        ARM_TARGET = target;
    }

    public double getArmTarget() {
        return ARM_TARGET;
    }
    public double getArmPos()  { return armMotor.getCurrentPosition(); }

    public void manual(double n) {
        setArmTarget(ARM_TARGET + n * ARM_SPEED);
    }


    public void armLowBasket() {setArmTarget(ARM_LOWBASKET);}
    public void armIntake() {setArmTarget(ARM_INTAKE);}
    public void armSpecimen() {
        setArmTarget(ARM_SPECIMEN);
    }
    public void armObservation() {setArmTarget(ARM_OBSERVATION);}
    public void armMax() {setArmTarget(ARM_MAX);}
    public void armSpecimenScore() {setArmTarget(ARM_SPECIMEN_SCORE);}

    public void armClear() {setArmTarget(ARM_CLEAR);}

    public void resetEncoder() {
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armStart = armMotor.getCurrentPosition();
        ARM_TARGET = armAngle();
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }




}