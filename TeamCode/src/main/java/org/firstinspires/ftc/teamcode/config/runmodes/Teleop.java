//package org.firstinspires.ftc.teamcode.config.runmodes;
//
//import org.firstinspires.ftc.teamcode.R;
//import org.firstinspires.ftc.teamcode.config.subsystems.Arm;
//import org.firstinspires.ftc.teamcode.config.pedroPathing.follower.Follower;
//import org.firstinspires.ftc.teamcode.config.pedroPathing.localization.Pose;
//
//import com.qualcomm.robotcore.hardware.Gamepad;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//
//import org.firstinspires.ftc.robotcore.external.Telemetry;
//import org.firstinspires.ftc.teamcode.config.util.action.Action;
//import org.firstinspires.ftc.teamcode.config.util.action.Actions;
//import org.firstinspires.ftc.teamcode.config.util.action.InstantAction;
//import org.firstinspires.ftc.teamcode.config.util.action.ParallelAction;
//import org.firstinspires.ftc.teamcode.config.util.action.RunAction;
//import org.firstinspires.ftc.teamcode.config.util.action.SequentialAction;
//import org.firstinspires.ftc.teamcode.config.util.action.SleepAction;
//
//public class Teleop {
//
//    private Arm arm;
//
//
//    private Follower follower;
//    private Pose startPose;
//
//    private Telemetry telemetry;
//
//    private Gamepad gamepad1;
//    private Gamepad gamepad2;
//    private Gamepad currentGamepad1 = new Gamepad();
//    private Gamepad currentGamepad2 = new Gamepad();
//    private Gamepad previousGamepad1 = new Gamepad();
//    private Gamepad previousGamepad2 = new Gamepad();
//
//    public double speed = 0.75;
//    private boolean fieldCentric = true;
//
//
//    public Teleop(HardwareMap hardwareMap, Telemetry telemetry, Follower follower, Pose startPose,  boolean fieldCentric, Gamepad gamepad1, Gamepad gamepad2) {
//        arm = new Arm(hardwareMap, telemetry);
//        this.follower = follower;
//        this.startPose = startPose;
//
//        this.fieldCentric = fieldCentric;
//        this.telemetry = telemetry;
//        this.gamepad1 = gamepad1;
//        this.gamepad2 = gamepad2;
//    }
//
//    public RunAction stopDrive = new RunAction(this::stopDrive);
//    public RunAction startDrive = new RunAction(this::startDrive);
//
//    private void startDrive() {
//        follower.startTeleopDrive();
//    }
//
//    private void stopDrive(){
//        follower.breakFollowing();
//    }
//
//
//    public void init() {
//        arm.init();
//    }
//
//    public void update() {
//        previousGamepad1.copy(currentGamepad1);
//        previousGamepad2.copy(currentGamepad2);
//        currentGamepad1.copy(gamepad1);
//        currentGamepad2.copy(gamepad2);
//
//        if (gamepad1.right_bumper)
//            speed = 1;
//        else if (gamepad1.left_bumper)
//            speed = 0.25;
//        else
//            speed = 0.75;
//
//        arm.manual(gamepad2.right_trigger - gamepad2.left_trigger);
//
//        follower.setTeleOpMovementVectors(-gamepad1.left_stick_y * speed, -gamepad1.left_stick_x * speed, -gamepad1.right_stick_x * speed, fieldCentric);
//        follower.update();
//
//        telemetry.addData("X", follower.getPose().getX());
//        telemetry.addData("Y", follower.getPose().getY());
//        telemetry.addData("Heading", Math.toDegrees(follower.getPose().getHeading()));
//
//        telemetry.addData("Actual Arm Pos", arm.arm.getCurrentPosition());
//        telemetry.update();
//    }
//
//    public void start() {
//        arm.start();
//        follower.setPose(startPose);
//        follower.startTeleopDrive();
//    }
//
//
//
//}