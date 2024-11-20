package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;



import org.firstinspires.ftc.teamcode.config.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.config.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.config.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.config.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.config.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.config.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.config.pedroPathing.util.Timer;
import org.firstinspires.ftc.teamcode.config.subsystems.Arm;
import org.firstinspires.ftc.teamcode.config.subsystems.EndEffector;
import org.firstinspires.ftc.teamcode.config.util.action.Actions;
import org.firstinspires.ftc.teamcode.config.util.action.SleepAction;

@Autonomous (name = "Gortimus Maximus Cheese Auto")
public class slightlyLessCheeseAuto extends OpMode {

    private Follower follower;
    private Timer pathTimer;
    private int pathState;
    private Arm arm;
    private EndEffector endEffector;

    // Define key poses (x, y, and heading values not set)
    private Pose startPos = new Pose(10, 43, Math.toRadians(180));
    private Pose controlPointSample1Pos = new Pose(16.6, 32.5, Math.toRadians(180));
    private Pose controlPointSampleone2Pos = new Pose(93, 22, Math.toRadians(180));
    private Pose behindSample1Pos = new Pose(67, 29, Math.toRadians(180));
    private Pose ObsSample1ControlPoint1Pos = new Pose(93, 22, Math.toRadians(180));
    private Pose ObsSample1Pos = new Pose(14, 17, Math.toRadians(180));
    private Pose controlPointSample2Pos = new Pose(85, 26, Math.toRadians(0));
    private Pose behindSample2Pos = new Pose(65, 12, Math.toRadians(180));
    private Pose ObsSample2Pos = new Pose(14, 12, Math.toRadians(180));
    private Pose backPos = new Pose(25, 12, Math.toRadians(180));
    private Pose turnPos = new Pose (23,19,Math.toRadians(0));
    private Pose intakeObsPos = new Pose(15, 19, Math.toRadians(0));
    private Pose lineUpSpecimen1Pos = new Pose(30, 66, Math.toRadians(180));
    private Pose scoreSpecimen1Pos = new Pose(47, 65, Math.toRadians(180));
    private Pose scoreSlide = new Pose(45,71, Math.toRadians(180));
    private Pose intakeSpecimen2 = new Pose(35,19,Math.toRadians(0));
//    private Pose hangSpecimen2Pos = new Pose(64, 33, Math.toRadians(180));
//    private Pose hangSpecimen3Pos = new Pose(64, 33, Math.toRadians(180));
//    private Pose hangSpecimen4Pos = new Pose(64, 33, Math.toRadians(180));
//    private Pose parkPos = new Pose(64, 33, Math.toRadians(180));

    private PathChain intakeSPecimen2,behindSample1Path, pushSample1Path, behindSample2Path, pushSample2Path, backPath, turnPath, intakeObs1Path, lineUpSpecimen1Path, scoreSpecimen1Path,slidePath, driveToObsPath,intakeObs2Path, hangSpecimen2Path, intakeObs3Path, hangSpecimen3Path, intakeObs4Path, hangSpecimen4Path, parkPath;
    public void buildPaths() {
        behindSample1Path = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(startPos), new Point(controlPointSample1Pos), new Point(behindSample1Pos)))
                .setConstantHeadingInterpolation(behindSample1Pos.getHeading())
                .build();
        pushSample1Path = follower.pathBuilder()
//                .addPath(new BezierCurve(new Point(behindSample1Pos), new Point(ObsSample1ControlPoint1Pos), new Point(controlPointSampleone2Pos), new Point(ObsSample1Pos)))
                .addPath(new BezierCurve(new Point(behindSample1Pos),    new Point(ObsSample1Pos)))
                .setConstantHeadingInterpolation(ObsSample1Pos.getHeading())
                .build();
        behindSample2Path = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(ObsSample1Pos), new Point(controlPointSample2Pos), new Point(behindSample2Pos)))
                .setConstantHeadingInterpolation(behindSample2Pos.getHeading())
                .build();
        pushSample2Path = follower.pathBuilder()
                .addPath(new BezierLine(new Point(behindSample2Pos), new Point(ObsSample2Pos)))
                .setConstantHeadingInterpolation(ObsSample2Pos.getHeading())
                .build();
        backPath = follower.pathBuilder()
                .addPath(new BezierLine(new Point(ObsSample2Pos), new Point(backPos)))
                .setConstantHeadingInterpolation(backPos.getHeading())
                .build();
        turnPath = follower.pathBuilder()
                .addPath(new BezierLine(new Point(backPos), new Point(turnPos)))
                .setConstantHeadingInterpolation(turnPos.getHeading())
                .build();
        intakeObs1Path = follower.pathBuilder()
                .addPath(new BezierLine(new Point(turnPos), new Point(intakeObsPos)))
                .setConstantHeadingInterpolation(intakeObsPos.getHeading())
                .build();
        lineUpSpecimen1Path = follower.pathBuilder()
                .addPath(new BezierLine(new Point(intakeObsPos), new Point(lineUpSpecimen1Pos)))
                .setConstantHeadingInterpolation(lineUpSpecimen1Pos.getHeading())
                .build();
        scoreSpecimen1Path = follower.pathBuilder()
                .addPath(new BezierLine(new Point(lineUpSpecimen1Pos), new Point(scoreSpecimen1Pos)))
                .setConstantHeadingInterpolation(scoreSpecimen1Pos.getHeading())
                .build();
        slidePath = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scoreSpecimen1Pos), new Point(scoreSlide)))
                .setConstantHeadingInterpolation(scoreSlide.getHeading())
                .build();
        driveToObsPath = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scoreSlide), new Point(turnPos)))
                .setConstantHeadingInterpolation(scoreSlide.getHeading())
                .build();
        intakeSPecimen2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(turnPos), new Point(intakeSpecimen2)))
                .setConstantHeadingInterpolation(scoreSlide.getHeading())
                .build();


    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                Actions.runBlocking(endEffector.openClaw);
                Actions.runBlocking(endEffector.diffyInit);
                setPathState(1);
                break;
            case 1:
                if (!follower.isBusy()) {
                    follower.followPath(behindSample1Path);
                 //   Actions.runBlocking(new SleepAction(1));
                    setPathState(2);
                }
                break;
            case 2:
                if (!follower.isBusy()) {
                    follower.followPath(pushSample1Path);
             //       Actions.runBlocking(new SleepAction(1));
                 //   follower.setMaxPower(1);
                    setPathState(3);
                }
                break;
            case 3:
                if (!follower.isBusy()) {
                   // follower.setMaxPower(0.4);
                    follower.followPath(behindSample2Path);
             //       Actions.runBlocking(new SleepAction(1));
                  //  follower.setMaxPower(0.8);
                    setPathState(4);
                }
                break;
            case 4:
                if (!follower.isBusy()) {
                    follower.followPath(pushSample2Path);
             //       Actions.runBlocking(new SleepAction(1));
                    setPathState(5);
                }
                break;
            case 5:
                if (!follower.isBusy()) {
                    follower.followPath(backPath);
             //       Actions.runBlocking(new SleepAction(1));
                    setPathState(6);
                }
                break;
            case 6:
                if (!follower.isBusy()) {
                    follower.followPath(turnPath);
               //     Actions.runBlocking(new SleepAction(1));
                    setPathState(7);
                }
                break;
            case 7:
                if (!follower.isBusy()) {
                    Actions.runBlocking(arm.armWallIntakeFinal);
                    Actions.runBlocking(endEffector.diffyWall);
                    follower.followPath(intakeObs1Path);
                    Actions.runBlocking(new SleepAction(1));
                    setPathState(8);
                }
                break;
            case 8:
                if (!follower.isBusy()  || pathTimer.getElapsedTimeSeconds() > 1) {
                    Actions.runBlocking(endEffector.closeClaw);
                    follower.followPath(lineUpSpecimen1Path);
                    Actions.runBlocking(arm.autoArmPreSpecimen);
                    Actions.runBlocking(new SleepAction(1));
                    setPathState(9);
                }
                break;
            case 9:
                if (!follower.isBusy()) {
                    Actions.runBlocking(endEffector.autoSpecimen);
                    follower.followPath(scoreSpecimen1Path);
                    Actions.runBlocking(new SleepAction(1));
                    setPathState(11);
                }
                break;

            case 11:
                if  (!follower.isBusy()  || pathTimer.getElapsedTimeSeconds() > 1) {
                    follower.followPath(slidePath);
                    Actions.runBlocking(new SleepAction(1));
                    setPathState(12);
                }
                break;

            case 12:
                if (!follower.isBusy()) {
                   Actions.runBlocking(endEffector.openClaw);
                    follower.followPath(driveToObsPath);
                    Actions.runBlocking(new SleepAction(1));
                    setPathState(69);
                }
                break;
            case 69:
                if  (!follower.isBusy()) {
                    Actions.runBlocking(endEffector.diffyWall2);
                    Actions.runBlocking(arm.autoArmIntake2);
                    follower.followPath(intakeSPecimen2);
                    Actions.runBlocking(new SleepAction(1));
                    setPathState(13);
            }
            break;
            case 13:
                if (!follower.isBusy()  || pathTimer.getElapsedTimeSeconds() > 1) {
                    Actions.runBlocking(endEffector.closeClaw);
                    follower.followPath(lineUpSpecimen1Path);
                    Actions.runBlocking(arm.autoArmPreSpecimen);
                    Actions.runBlocking(new SleepAction(1));
                    setPathState(14);
                }
                break;
            case 14:
                if (!follower.isBusy()) {
                    Actions.runBlocking(endEffector.autoSpecimen);
                    follower.followPath(scoreSpecimen1Path);
                    Actions.runBlocking(new SleepAction(1));
                    setPathState(15);
                }
                break;

            case 15:
                if  (!follower.isBusy()  || pathTimer.getElapsedTimeSeconds() > 1) {
                    follower.followPath(slidePath);
                    Actions.runBlocking(new SleepAction(1));
                    setPathState(100);
                }
                break;
            case 100:
                if (!follower.isBusy()) {
                    setPathState(-1);
                }
                break;
            default:
                requestOpModeStop();
                break;
        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    @Override
    public void loop() {
        follower.update();
        arm.update();
        autonomousPathUpdate();

        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", Math.toRadians(follower.getPose().getHeading()));
        telemetry.addData("Arm Target", arm.getArmTarget());
        telemetry.addData("Arm Pos", arm.getCurrentPosition());
        telemetry.addData("Diffy1 Position", "%.2f", endEffector.getDiffy1Position());
        telemetry.addData("Diffy2 Position", "%.2f", endEffector.getDiffy2Position());
        telemetry.update();
    }

    @Override
    public void init() {
        pathTimer = new Timer();
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPos);
        arm = new Arm(hardwareMap);
        endEffector = new EndEffector(hardwareMap);
        endEffector.openClaw();
        endEffector.initPosition();
        follower.setMaxPower(0.85);
        buildPaths();
    }

    @Override
    public void start() {
        pathTimer.resetTimer();
        setPathState(0);
    }

}
