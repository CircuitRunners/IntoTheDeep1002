package org.firstinspires.ftc.teamcode.config.subsystems;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
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
    public RunAction passthroughSpecimenScorePosition, scoreBucketDiffy, diffyWall2,openClaw, openClawAuto, closeClaw, diffyIdle, diffyInit, diffyIntakeH, diffyIntakeV, diffyIntakeAL, diffyIntakeAR, diffySpecimen, diffyBasket, diffyObs, diffyHang, diffyClear, diffyScore, autoPreSpecimen, autoSpecimen, diffyWall;

    public EndEffector(HardwareMap hardwareMap) {
        claw = hardwareMap.get(Servo.class, HWValues.CLAW);
        diffy1 = hardwareMap.get(Servo.class, HWValues.DIFFY1);
        diffy2 = hardwareMap.get(Servo.class, HWValues.DIFFY2);

        openClaw = new RunAction(this::openClaw);
        closeClaw = new RunAction(this::closeClaw);
        openClawAuto = new RunAction(this::openClawAuto);

        diffyIdle = new RunAction(this::idlePosition);
        diffyInit = new RunAction(this::initPosition);
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
        autoPreSpecimen = new RunAction(this::autoPreSpecimen);
        autoSpecimen = new RunAction(this::autoSpecimen);
        diffyWall = new RunAction(this::diffyWall);
        diffyWall2 = new RunAction(this::diffyWall2);
        scoreBucketDiffy = new RunAction(this::scoreBucketDiffy);
        passthroughSpecimenScorePosition = new RunAction(this::passthroughSpecimenScorePosition);
        // openClaw();
        //idlePosition();
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
        claw.setPosition(0.50);
    }

    public void closeClaw() {
        claw.setPosition(0.8);
    }
    public void openClawAuto() {
        claw.setPosition(0.1);
    }
    public void switchClaw() {
        if (claw.getPosition() < 0.6) {
            closeClaw();
        } else {
            openClaw();
        }
    }
    public void idlePosition() {
            diffy1.setPosition(0.41);
            diffy2.setPosition(0.24);
    }

    public void initPosition() {
        diffy1.setPosition(0.28);
        diffy2.setPosition(0.12);
    }

    public void intakeClear() {
        diffy1.setPosition(0.38);
        diffy2.setPosition(0.20);
    }
    public void wallGameDiffy(){
        diffy1.setPosition(0.49);
        diffy2.setPosition(0.32);
    }

    public void intakePositionH() {
        diffy1.setPosition(0.36);
        diffy2.setPosition(0.18);
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
        diffy1.setPosition(0.63);
        diffy2.setPosition(0.46);
    }
    public void passthroughSpecimenScorePosition(){
        diffy1.setPosition(0.98);
        diffy2.setPosition(0.02);


    }
    public void basketPosition() {
        diffy1.setPosition(0.53);
        diffy2.setPosition(0.35);
    }
    public void obsPosition() {
        diffy1.setPosition(0.45);
        diffy2.setPosition(0.28);
    }

    public void hangPosition() {
        diffy1.setPosition(0.5);
        diffy2.setPosition(0.5);
    }

//    public void autoPreSpecimen() {
//        diffy1.setPosition(0.49);
//        diffy2.setPosition(0.37);
//    }

    public void autoSpecimen() {
        diffy1.setPosition(0.45);
        diffy2.setPosition(0.3);

    }

    public void autoPreSpecimen(){
        diffy1.setPosition(0.49);
        diffy2.setPosition(0.32);
    }

    public void diffyWall(){
        diffy1.setPosition(0.53);
        diffy2.setPosition(0.40);
    }
    public void diffyWall2(){
        diffy1.setPosition(0.55);
        diffy2.setPosition(0.38);
    }

    public void scoreBucketDiffy(){
        diffy1.setPosition(0.59);
        diffy2.setPosition(0.41);
    }


    public void diffy1Set(double n){
        diffy1.setPosition(n);
    }
    public void diffy1increment() {diffy1.setPosition(diffy1.getPosition() + 0.02);}
    public void diffy2increment() {diffy2.setPosition(diffy2.getPosition() + 0.02);}
    public void diffy1decrement() {diffy1.setPosition(diffy1.getPosition() - 0.02);}
    public void diffy2decrement() {diffy2.setPosition(diffy2.getPosition() - 0.02);}
    public void diffy2Set(double n) {
        diffy2.setPosition(n);
    }




}
