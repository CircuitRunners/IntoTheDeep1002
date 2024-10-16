package org.firstinspires.ftc.teamcode.config.subsystems;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.config.util.HWValues;

import org.firstinspires.ftc.teamcode.config.util.action.Action;
import org.firstinspires.ftc.teamcode.config.util.action.Actions;
import org.firstinspires.ftc.teamcode.config.util.action.ParallelAction;
import org.firstinspires.ftc.teamcode.config.util.action.RunAction;
import org.firstinspires.ftc.teamcode.config.util.action.SequentialAction;
import org.firstinspires.ftc.teamcode.config.util.action.SleepAction;



public class EndEffector {
    private final Servo claw;
    private final Servo diffy1;
    private final Servo diffy2;

    // TODO
    public RunAction openClaw, closeClaw, diffyIdle, diffyIntakeH, diffyIntakeV, diffyIntakeAL, diffyIntakeAR, diffySpecimen, diffyBasket, diffyObs, diffyHang, diffyClear, diffyScore;

    public EndEffector(HardwareMap hardwareMap) {
        claw = hardwareMap.get(Servo.class, HWValues.CLAW);
        diffy1 = hardwareMap.get(Servo.class, HWValues.DIFFY1);
        diffy2 = hardwareMap.get(Servo.class, HWValues.DIFFY2);

        openClaw = new RunAction(this::openClaw);
        closeClaw = new RunAction(this::closeClaw);

        diffyIdle = new RunAction(this::idlePosition);
        diffyIntakeH = new RunAction(this::intakePositionH);
        diffyIntakeV = new RunAction(this::intakePositionV);
        diffyIntakeAL = new RunAction(this::intakePositionAL);
        diffyIntakeAR = new RunAction(this::intakePositionAR);
        diffySpecimen = new RunAction(this::specimenPosition);
        diffyBasket = new RunAction(this::basketPosition);
        diffyObs = new RunAction(this::obsPosition);
        diffyHang = new RunAction(this::hangPosition);
        diffyClear = new RunAction(this::intakeClear);
        diffyScore = new RunAction(this::specimenScorePosition);

        openClaw();
        idlePosition();
    }

    public double getClawPosition() {
        return claw.getPosition();
    }

    public double getDiffy1Position() {
        return diffy1.getPosition();
    }
    public double getDiffy2Position() {
        return diffy2.getPosition();
    }

    public void openClaw() {
        claw.setPosition(0);
    }

    public void closeClaw() {
        claw.setPosition(0.65);
    }
    public void switchClaw() {
        if (claw.getPosition() < 0.60) {
            closeClaw();
        } else {
            openClaw();
        }
    }
    public void idlePosition() {
        diffy1.setPosition(0.30);
        diffy2.setPosition(0.26);
    }

    public void intakeClear() {
        diffy1.setPosition(0.38);
        diffy2.setPosition(0.28);
    }

    public void intakePositionH() {
        diffy1.setPosition(0.39);
        diffy2.setPosition(0.28);
    }
    public void intakePositionV() {
        diffy1.setPosition(0.44);
        diffy2.setPosition(0.98);
    }
    public void intakePositionAL() {
        diffy1.setPosition(0.52);
        diffy2.setPosition(0.89);
    }
    public void intakePositionAR() {
        diffy1.setPosition(0.75);
        diffy2.setPosition(0.66);
    }

    public void specimenPosition() {
        diffy1.setPosition(0.57);
        diffy2.setPosition(0.56);
    }

    public void specimenScorePosition() {
        diffy1.setPosition(0.57);
        diffy2.setPosition(0.56);
    }
    public void basketPosition() {
        diffy1.setPosition(0.53);
        diffy2.setPosition(0.47);
    }
    public void obsPosition() {
        diffy1.setPosition(0.45);
        diffy2.setPosition(0.4);
    }

    public void hangPosition() {
        diffy1.setPosition(0.5);
        diffy2.setPosition(0.5);
    }

    public void diffy1Set(double n){
        diffy1.setPosition(n);
    }
    public void diffy1increment() {diffy1.setPosition(diffy1.getPosition() + 0.01);}
    public void diffy2increment() {diffy2.setPosition(diffy2.getPosition() + 0.01);}
    public void diffy1decrement() {diffy1.setPosition(diffy1.getPosition() - 0.01);}
    public void diffy2decrement() {diffy2.setPosition(diffy2.getPosition() - 0.01);}
    public void diffy2Set(double n) {
        diffy2.setPosition(n);
    }




}
