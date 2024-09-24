//package org.firstinspires.ftc.teamcode.opmode;
//
//
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//
//import org.firstinspires.ftc.teamcode.config.pedroPathing.follower.Follower;
//import org.firstinspires.ftc.teamcode.config.pedroPathing.localization.Pose;
//import org.firstinspires.ftc.teamcode.config.pedroPathing.pathGeneration.BezierCurve;
//import org.firstinspires.ftc.teamcode.config.pedroPathing.pathGeneration.BezierLine;
//import org.firstinspires.ftc.teamcode.config.pedroPathing.pathGeneration.PathChain;
//import org.firstinspires.ftc.teamcode.config.pedroPathing.pathGeneration.Point;
//import org.firstinspires.ftc.teamcode.config.pedroPathing.util.Timer;
//
//@Autonomous (name = "Test Auto Path")
//public class testAuto extends OpMode {
//
//    private Follower follower;
//    private Timer pathTimer;
//    private int pathState;
//
//
//    private Pose startPoint = new Pose(9.757, 84.98,0);
//    private Pose chamber = new Pose(36.784,77.275,0);
//    private Pose spikeMark1 = new Pose(53.608, 114.059, Math.toRadians(180));
//    private Pose  net1 = new Pose(14.828,129.743, Math.toRadians(180));
//    private Pose spikeMark2 = new Pose(69.58,123.75, Math.toRadians(180));
//    private Pose net2 = new Pose(17.96,133.16, Math.toRadians(180));
//    private Pose spikeMark3 = new Pose(66.725,134.305, Math.toRadians(180));
//    private Pose net3 = new Pose(18,136.871, Math.toRadians(180));
//    private Pose observation = new Pose(18,18.250, Math.toRadians(180));
//
//   private PathChain pathChain;
//   public void buildPaths(){
//       pathChain = follower.pathBuilder()
//               .addPath(new BezierLine(new Point(startPoint), new Point(chamber)))
//               .setConstantHeadingInterpolation(chamber.getHeading())
//               .addPath(new BezierCurve(new Point(chamber), new Point(27.66,104.93, Point.CARTESIAN), new Point (75.564,108.642, Point.CARTESIAN),new Point(spikeMark1)))
//               .setConstantHeadingInterpolation(spikeMark1.getHeading())
//               .addPath(new BezierLine(new Point(spikeMark1), new Point(net1)))
//               .setConstantHeadingInterpolation(net1.getHeading())
//               .addPath(new BezierCurve(new Point(net1),new Point(48.19,111.49, Point.CARTESIAN), new Point(spikeMark2)))
//               .setConstantHeadingInterpolation(spikeMark2.getHeading())
//               .addPath(new BezierLine(new Point(spikeMark2), new Point(net2)))
//               .setConstantHeadingInterpolation(net2.getHeading())
//               .addPath(new BezierCurve(new Point(net2),new Point(41.632,120.903, Point.CARTESIAN), new Point(spikeMark3)))
//               .setConstantHeadingInterpolation(spikeMark3.getHeading())
//               .addPath(new BezierLine(new Point(spikeMark3), new Point(net3)))
//               .setConstantHeadingInterpolation(net3.getHeading())
//               .addPath(new BezierLine(new Point(net3), new Point(observation)))
//               .setConstantHeadingInterpolation(observation.getHeading())
//
//
//               .build();
//   }
//
//    public void setPathState(int pState) {
//        pathState = pState;
//        pathTimer.resetTimer();
//    }
//    public void autonomousPathUpdate() {
//        switch (pathState) {
//            case 0:
//                follower.followPath(pathChain);
//                setPathState(1);
//                break;
//            case 1:
//                if (!follower.isBusy()) {
//                    setPathState(-1);
//                }
//                break;
//            default:
//                requestOpModeStop();
//                break;
//
//        }
//    }
//
//
//
//    @Override
//    public void loop(){
//        follower.update();
//        autonomousPathUpdate();
//
//        telemetry.addData("path state", pathState);
//        telemetry.addData("x", follower.getPose().getX());
//        telemetry.addData("y", follower.getPose().getY());
//        telemetry.addData("heading", follower.getPose().getHeading());
//        telemetry.update();
//    }
//
//
//
//    @Override
//    public void init(){
//        pathTimer = new Timer();
//        follower = new Follower(hardwareMap);
//        follower.setPose(startPoint);
//        buildPaths();
//
//    }
//
//    @Override
//    public void start(){
//        pathTimer.resetTimer();
//        setPathState(0);
//
//    }
//
//}
