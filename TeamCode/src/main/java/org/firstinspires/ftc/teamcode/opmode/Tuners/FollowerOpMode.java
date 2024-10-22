package org.firstinspires.ftc.teamcode.opmode.Tuners;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.config.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.config.pedroPathing.localization.Pose;

@TeleOp (name = "FollowerTeleOp", group = "test")
public class FollowerOpMode extends OpMode{
    private Follower follower;

    private Pose startPosition = new Pose(8, 85, 0);

    @Override
    public void loop(){
        follower.update();

        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    @Override
    public void init(){
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPosition);
    }


}
