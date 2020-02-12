
package org.firstinspires.ftc.teamcode;

import android.content.Context;

import com.acmerobotics.roadrunner.followers.RamseteFollower;
import com.acmerobotics.roadrunner.path.Path;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.TrajectoryGenerator;
import com.acmerobotics.roadrunner.trajectory.TrajectoryLoader;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.io.File;

// autonomous program that drives bot forward a set distance, stops then
// backs up to the starting point using encoders to measure the distance.
// This example assumes there is one encoder, attached to the left motor

@Autonomous(name="Road runner blocks red")
//@disabled
public class RoadRunnerRedBlock extends LinearOpMode
{
    RobotRoadRunner robot;

    @Override
    public void runOpMode() throws InterruptedException
    {
        robot = new RobotRoadRunner(hardwareMap, this);

        //robotMecanum.initiateVuforia();

        robot.robotMecanum.gyro.calibrate();

        //==========================================================================================
        //Pre init

        robot.robotMecanum.hooks(true);
        robot.robotMecanum.claw(true);

        telemetry.addData("Step 1", "init finished");
        telemetry.update();

        waitForStart();

        RamseteFollower follower = new RamseteFollower(2.0, 0.7);

        try {
            follower.followTrajectory(TrajectoryLoader.load(hardwareMap.appContext.getAssets()));
        }catch (Exception e){
            telemetry.addData("Error", e);
        }

        while(follower.isFollowing() && opModeIsActive()){
            robot.drive.setDriveSignal(follower.update(robot.drive.getPoseEstimate()));
        }



    }


}

