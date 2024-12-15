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
import org.firstinspires.ftc.teamcode.config.util.action.Action;
import org.firstinspires.ftc.teamcode.config.util.action.Actions;
import org.firstinspires.ftc.teamcode.config.util.action.SleepAction;

@Autonomous (name = "lm3 cooking trust")
public class slightlyLessCheeseAuto extends OpMode {

    private Follower follower;
    private Timer pathTimer;
    private int pathState;
    private Arm arm;
    private EndEffector endEffector;

    // Define key poses (x, y, and heading values not set)
    private Pose startPos = new Pose(10, 43, Math.toRadians(180));
    private Pose controlPointSample1Pos = new Pose(16.6, 32.5, Math.toRadians(180));
    private Pose controlPointSampleone2Pos = new Pose(93, 20, Math.toRadians(180));
    private Pose behindSample1Pos = new Pose(67, 29, Math.toRadians(180));
    private Pose ObsSample1ControlPoint1Pos = new Pose(93, 22, Math.toRadians(180));
    private Pose ObsSample1Pos = new Pose(14.9, 16, Math.toRadians(180));
    private Pose controlPointSample2Pos = new Pose(85, 26, Math.toRadians(0));
    private Pose behindSample2Pos = new Pose(65, 12, Math.toRadians(180));
    private Pose ObsSample2Pos = new Pose(15, 12, Math.toRadians(180));
    private Pose backPos = new Pose(25, 16, Math.toRadians(180));
    private Pose turnPos = new Pose (23,19,Math.toRadians(0));
    private Pose intakeObsPos = new Pose(13.8, 19, Math.toRadians(0));
    private Pose intakeObsPos2 = new Pose(14, 19, Math.toRadians(0));
    private Pose lineUpSpecimen1Pos = new Pose(30, 66, Math.toRadians(180));
    private Pose scoreSpecimen1Pos = new Pose(50, 65, Math.toRadians(180));
    private Pose scoreSlide = new Pose(42,69, Math.toRadians(180));
    private Pose intakeSpecimen2 = new Pose(35,19,Math.toRadians(0));
    private Pose mergePos = new Pose(30, 50, Math.toRadians(180));
    private Pose parkPos = new Pose(15, 30, Math.toRadians(180));

    private PathChain behindSample1Path, pushSample1Path, behindSample2Path, pushSample2Path, backPath, turnPath, intakeObs1Path, lineUpSpecimen1Path, scoreSpecimen1Path, slidePath, driveToObsPath, intakeSpecimen2Path, mergePath, hangSpecimen2Path, intakeObs3Path, hangSpecimen3Path, intakeObs4Path, hangSpecimen4Path, parkPath;
    public void buildPaths() {
        behindSample1Path = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(startPos), new Point(controlPointSample1Pos), new Point(behindSample1Pos)))
                .setConstantHeadingInterpolation(behindSample1Pos.getHeading())
                .build();
        pushSample1Path = follower.pathBuilder()
//                .addPath(new BezierCurve(new Point(behindSample1Pos), new Point(ObsSample1ControlPoint1Pos), new Point(controlPointSampleone2Pos), new Point(ObsSample1Pos)))
                .addPath(new BezierCurve(new Point(behindSample1Pos), new Point(ObsSample1Pos)))
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
        intakeSpecimen2Path = follower.pathBuilder()
                .addPath(new BezierLine(new Point(turnPos), new Point(intakeObsPos2)))
                .setConstantHeadingInterpolation(intakeObsPos2.getHeading())
                .build();
        mergePath = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scoreSlide), new Point(mergePos)))
                .setConstantHeadingInterpolation(scoreSlide.getHeading())
                .addPath(new BezierLine(new Point(mergePos), new Point(turnPos)))
                .setConstantHeadingInterpolation(turnPos.getHeading())
                .build();
        parkPath = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scoreSlide), new Point(parkPos)))
                .setConstantHeadingInterpolation(parkPos.getHeading())
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
                    setPathState(5);
                }
                break;
            case 3:
                if (!follower.isBusy()) {
                    follower.followPath(behindSample2Path);
             //       Actions.runBlocking(new SleepAction(1));
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
                    Actions.runBlocking(endEffector.openClaw);
                    Actions.runBlocking(endEffector.diffyWall);
                    Actions.runBlocking(arm.armIntermediate);
                    follower.followPath(turnPath);
                //    Actions.runBlocking(new SleepAction(1));
                    setPathState(7);
                }
                break;
            case 7:
                if (!follower.isBusy()) {
                    Actions.runBlocking(arm.armWallIntakeFinal);
                    follower.followPath(intakeObs1Path,true);
                //    Actions.runBlocking(new SleepAction(1));
                    setPathState(8);
                }
                break;
            case 8:
                if (!follower.isBusy() || pathTimer.getElapsedTimeSeconds() > 1) {
                    Actions.runBlocking(endEffector.closeClaw);
                    //follower.followPath(lineUpSpecimen1Path);
                    //Actions.runBlocking(arm.autoArmPreSpecimen);
                    //    Actions.runBlocking(new SleepAction(1));
                    setPathState(80);
                }
                break;
            case 80:
                if (!follower.isBusy()  || pathTimer.getElapsedTimeSeconds() > 0.75) {
                    //Actions.runBlocking(endEffector.closeClaw);
                    follower.followPath(lineUpSpecimen1Path);
                    Actions.runBlocking(arm.autoArmPreSpecimen);
                    Actions.runBlocking(endEffector.autoSpecimen);
                    //    Actions.runBlocking(new SleepAction(1));
                    setPathState(9);
                }
                break;
            case 9:
                if (!follower.isBusy()) {

                    follower.followPath(scoreSpecimen1Path);
                //    Actions.runBlocking(new SleepAction(1));
                    setPathState(11);
                }
                break;
            case 11: //scored specimen 1
                if  (!follower.isBusy()  || pathTimer.getElapsedTimeSeconds() > 0.6) {
                    follower.followPath(slidePath);
                //    Actions.runBlocking(new SleepAction(1));
                    setPathState(12);
                }
                break;
            case 12:
                if  (!follower.isBusy() || pathTimer.getElapsedTimeSeconds() > 0.6) {
                    Actions.runBlocking(endEffector.openClaw);
                  //  Actions.runBlocking(arm.autoArmSpecimenDown);
                    follower.followPath(mergePath);
                    Actions.runBlocking(arm.armIntermediate);
                    Actions.runBlocking(endEffector.diffyWall);
                //    Actions.runBlocking(new SleepAction(1));
                    setPathState(69);
                }
                break;
            case 69:
                if  (!follower.isBusy()) {
                    Actions.runBlocking(arm.armWallIntakeFinal);
                    follower.followPath(intakeSpecimen2Path, true);
                //    Actions.runBlocking(new SleepAction(1));
                    setPathState(13);
                }
                break;
            case 13:
                if (!follower.isBusy() || pathTimer.getElapsedTimeSeconds() > 0.9) {
                    Actions.runBlocking(endEffector.closeClaw);
                    //follower.followPath(lineUpSpecimen1Path);
                    //Actions.runBlocking(arm.autoArmPreSpecimen);
                    //    Actions.runBlocking(new SleepAction(1));
                    setPathState(81);
                }
                break;
            case 81:
                if (!follower.isBusy() || pathTimer.getElapsedTimeSeconds() > 0.75) {
                    follower.followPath(lineUpSpecimen1Path);
                    Actions.runBlocking(arm.autoArmPreSpecimen);
                    Actions.runBlocking(endEffector.autoSpecimen);
                    //    Actions.runBlocking(new SleepAction(1));
                    setPathState(14);
                }
                break;
            case 14:
                if (!follower.isBusy()) {

                    follower.followPath(scoreSpecimen1Path);
                //    Actions.runBlocking(new SleepAction(1));
                    setPathState(15);
                }
                break;
            case 15: //scored specimen 2
                if  (!follower.isBusy() || pathTimer.getElapsedTimeSeconds() > 0.6) {
                    follower.followPath(slidePath);
                //    Actions.runBlocking(new SleepAction(1));
                    setPathState(16);
                }
                break;
            case 16:
                if  (!follower.isBusy()  || pathTimer.getElapsedTimeSeconds() > 0.6) {
                    Actions.runBlocking(endEffector.openClaw);
                 //   Actions.runBlocking(arm.autoArmSpecimenDown);
                    follower.followPath(mergePath);
                    Actions.runBlocking(arm.armIntermediate);
                    Actions.runBlocking(endEffector.diffyWall);
                //    Actions.runBlocking(new SleepAction(1));
                    setPathState(17);
                }
                break;
            case 17:
                if  (!follower.isBusy()) {
                    Actions.runBlocking(arm.armWallIntakeFinal);
                    follower.followPath(intakeSpecimen2Path, true);
                 //   Actions.runBlocking(new SleepAction(1));
                    setPathState(18);
                }
                break;
            case 18:
                if (!follower.isBusy() || pathTimer.getElapsedTimeSeconds() > 0.85) {
                    Actions.runBlocking(endEffector.closeClaw);
                    //follower.followPath(lineUpSpecimen1Path);
                    //Actions.runBlocking(arm.autoArmPreSpecimen);
                    //    Actions.runBlocking(new SleepAction(1));
                    setPathState(82);
                }
                break;
            case 82:
                if (!follower.isBusy() || pathTimer.getElapsedTimeSeconds() > 0.75) {
                    follower.followPath(lineUpSpecimen1Path);
                    Actions.runBlocking(arm.autoArmPreSpecimen);
                    Actions.runBlocking(endEffector.autoSpecimen);
                    //    Actions.runBlocking(new SleepAction(1));
                    setPathState(19);
                }
                break;
            case 19:
                if (!follower.isBusy()) {

                    follower.followPath(scoreSpecimen1Path);
                //    Actions.runBlocking(new SleepAction(1));
                    setPathState(21);
                }
                break;
            case 20: //scored specimen 3
                if  (!follower.isBusy()  || pathTimer.getElapsedTimeSeconds() > 0.6) {
                    follower.followPath(slidePath);
                //    Actions.runBlocking(new SleepAction(1));
                    setPathState(21);
                }
                break;
            case 21:
                if  (!follower.isBusy()  || pathTimer.getElapsedTimeSeconds() > 0.6) {
                    Actions.runBlocking(endEffector.openClaw);
                    Actions.runBlocking(arm.autoArmSpecimenDown);
                    follower.followPath(parkPath);
                    setPathState(22);
                }
                break;
            case 22:
                if  (!follower.isBusy()) {
                    Actions.runBlocking(arm.armObservation);
                    Actions.runBlocking(endEffector.diffyInit);
                    Actions.runBlocking(endEffector.closeClaw);
                    setPathState(420);
                }
                break;
            case 420:
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
        endEffector.closeClaw();
        endEffector.initPosition();
        follower.setMaxPower(1);
        buildPaths();
    }

    @Override
    public void start() {
        pathTimer.resetTimer();
        setPathState(0);
    }

}
