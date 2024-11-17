//package org.firstinspires.ftc.teamcode.config.subsystems;
//
//import static org.firstinspires.ftc.teamcode.config.util.RobotConstants.ARM_CLEAR;
//import static org.firstinspires.ftc.teamcode.config.util.RobotConstants.ARM_INTAKE;
//import static org.firstinspires.ftc.teamcode.config.util.RobotConstants.ARM_LOWBASKET;
//import static org.firstinspires.ftc.teamcode.config.util.RobotConstants.ARM_MAX;
//import static org.firstinspires.ftc.teamcode.config.util.RobotConstants.ARM_OBSERVATION;
//import static org.firstinspires.ftc.teamcode.config.util.RobotConstants.ARM_OFF;
//import static org.firstinspires.ftc.teamcode.config.util.RobotConstants.ARM_SPECIMEN;
//import static org.firstinspires.ftc.teamcode.config.util.RobotConstants.ARM_SPECIMEN_SCORE;
//import static org.firstinspires.ftc.teamcode.config.util.RobotConstants.ARM_SPEED;
//import static org.firstinspires.ftc.teamcode.config.util.RobotConstants.ARM_TARGET;
//import static org.firstinspires.ftc.teamcode.config.util.RobotConstants.TICK_PER_RAD;
//import static org.firstinspires.ftc.teamcode.config.util.RobotConstants.armStart;
//import static org.firstinspires.ftc.teamcode.config.util.RobotConstants.f;
//import static org.firstinspires.ftc.teamcode.config.util.RobotConstants.kD;
//import static org.firstinspires.ftc.teamcode.config.util.RobotConstants.kI;
//import static org.firstinspires.ftc.teamcode.config.util.RobotConstants.kP;
//
//import com.qualcomm.robotcore.hardware.AnalogInput;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//
//import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
//import org.firstinspires.ftc.teamcode.config.pedroPathing.util.CustomPIDFCoefficients;
//import org.firstinspires.ftc.teamcode.config.pedroPathing.util.FeedForwardConstant;
//import org.firstinspires.ftc.teamcode.config.pedroPathing.util.PIDFController;
//import org.firstinspires.ftc.teamcode.config.util.action.RunAction;
//
//public class NewArm {
//    public static CustomPIDFCoefficients PID = new CustomPIDFCoefficients(kP, kI, kD, new NewArm.ArmPIDF());
//
//    static class ArmPIDF implements FeedForwardConstant {
//        @Override
//        public double getConstant(double input) {
//            return Math.sin(input) * f;
//        }
//    }
//
//    public DcMotorEx armMotor;
//    public AnalogInput enc;
//
//    public RunAction armLowBasket,armIntake,armSpecimen, armObservation, armMax, armSpecimenScore, armClear, armUpright, autoArmPreSpecimen, autoArmSpecimen;
//
//    public double armAngle() {
//        return (enc.getVoltage() / 3.2 * 360);
//    }
//
//    private PIDFController armPID = new PIDFController(PID);
//
//    public NewArm(HardwareMap hardwareMap) {
//        armMotor = hardwareMap.get(DcMotorEx.class, "arm");
//        enc = hardwareMap.get(AnalogInput.class, "enc");
//        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        armStart = armAngle();
//        ARM_TARGET = armAngle();
//
//        armLowBasket = new RunAction(this::armLowBasket);
//        armIntake = new RunAction(this::armIntake);
//        armSpecimen = new RunAction(this::armSpecimen);
//        armObservation = new RunAction(this::armObservation);
//        armMax = new RunAction(this::armMax);
//        armSpecimenScore = new RunAction(this::armSpecimenScore);
//        armClear = new RunAction(this::armClear);
//        armUpright = new RunAction(this::armUpright);
//        autoArmPreSpecimen = new RunAction(this::autoArmPreSpecimen);
//        autoArmSpecimen = new RunAction(this::autoArmSpecimen);
//    }
//
//    public void update() {
//        armPID.updateFeedForwardInput(armAngle());
//        armPID.setTargetPosition(ARM_TARGET);
//        armPID.updatePosition(armAngle());
//        armMotor.setPower(armPID.runPIDF());
//    }
//
//    public void setArmTarget(double target, double direction, boolean button) {
//        double currentDraw = armMotor.getCurrent(CurrentUnit.AMPS);
//
//        // If current exceeds the threshold, don't change the target to prevent damage
//        if (currentDraw > 1.5 && direction > 0 && !button) {
//            return; // Exit the method to prevent setting a new target
//        }
//        ARM_TARGET = target;
//    }
//    public void setArmTarget(double target) {
//        double currentDraw = armMotor.getCurrent(CurrentUnit.AMPS);
//        ARM_TARGET = target;
//    }
//    public double getArmCurrent() {return armMotor.getCurrent(CurrentUnit.AMPS);}
//
//    public double getArmTarget() {
//        return ARM_TARGET;
//    }
//    public double getArmPos()  { return armMotor.getCurrentPosition(); }
//
//    public void manual(double n) {
//        setArmTarget(ARM_TARGET + n * ARM_SPEED);
//    }
//    public void manual(double n, double direction, boolean button) {
//        setArmTarget(ARM_TARGET + n * ARM_SPEED, direction, button);
//    }
//
//
//    public void armLowBasket() {setArmTarget(ARM_LOWBASKET);}
//    public void armIntake() {setArmTarget(ARM_INTAKE);}
//    public void armSpecimen() {
//        setArmTarget(ARM_SPECIMEN);
//    }
//    public void armObservation() {setArmTarget(ARM_OBSERVATION);}
//    public void armMax() {setArmTarget(ARM_MAX);}
//    public void armSpecimenScore() {setArmTarget(ARM_SPECIMEN_SCORE);}
//
//    public void armClear() {setArmTarget(ARM_CLEAR);}
//    public void armUpright() {setArmTarget(-1.67);}
//    public void autoArmPreSpecimen() {setArmTarget(-0.36);}
//    public void autoArmSpecimen() {setArmTarget(0.2);}
//
//    public void resetEncoder() {
//        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        armStart = armMotor.getCurrentPosition();
//        ARM_TARGET = armAngle();
//        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//    }
//
//    public void scoreSpecimen() {
//        while (armMotor.getCurrent(CurrentUnit.AMPS) < 2) {
//            armMotor.setPower(-1);
//        }
//    }
//
//
//
//
//}