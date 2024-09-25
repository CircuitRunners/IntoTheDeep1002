package org.firstinspires.ftc.teamcode.config.subsystems;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.config.util.HWValues;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;


import org.firstinspires.ftc.teamcode.config.util.action.RunAction;


public class EndEffector {
    private final Servo claw;
    private final Servo diffy1;
    private final Servo diffy2;

    // TODO
    public RunAction actions;

    public EndEffector(HardwareMap hardwareMap) {
        claw = hardwareMap.get(Servo.class, HWValues.CLAW);
        diffy1 = hardwareMap.get(Servo.class, HWValues.DIFFY1);
        diffy2 = hardwareMap.get(Servo.class, HWValues.DIFFY2);
        closeClaw();
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
        claw.setPosition(0.5);
    }

    public void closeClaw() {
        claw.setPosition(0);
    }
    public void idlePosition() {
        diffy1.setPosition(0.32);
        diffy2.setPosition(0.52);
    }

    public void intakePositionH() {
        diffy1.setPosition(0.65);
        diffy2.setPosition(0.06);
    }
    public void intakePositionV() {
        diffy1.setPosition(0.44);
        diffy2.setPosition(0.27);
    }
    public void intakePositionAL() {
        diffy1.setPosition(0.38);
        diffy2.setPosition(0.34);
    }
    public void intakePositionAR() {
        diffy1.setPosition(0.14);
        diffy2.setPosition(0.59);
    }

    public void specimenPosition() {
        diffy1.setPosition(0.5);
        diffy2.setPosition(0.72);
    }
    public void basketPosition() {
        diffy1.setPosition(0.53);
        diffy2.setPosition(0.71);
    }
    public void obsPosition() {
        diffy1.setPosition(0.81);
        diffy2.setPosition(0.18);
    }

    public void hangPosition() {
        diffy1.setPosition(0.5);
        diffy2.setPosition(0.5);
    }

    public void diffy1Set(double n){
        diffy1.setPosition(n);
    }
    public void diffy2Set(double n) {
        diffy2.setPosition(n);
    }


}
