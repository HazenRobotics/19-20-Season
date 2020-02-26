
package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.followers.RamseteFollower;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.config.TrajectoryConfig;
import com.acmerobotics.roadrunner.*;
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumConstraints;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryConstraints;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.io.File;
import java.io.InputStream;

// autonomous program that drives bot forward a set distance, stops then
// backs up to the starting point using encoders to measure the distance.
// This example assumes there is one encoder, attached to the left motor

@Autonomous(name="Road runner blocks red")
//@disabled
public class RoadRunnerRedBlock extends LinearOpMode
{
    RobotRoadRunner robot;
    Trajectory trajectory;

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

        /*try{
            trajectory = TrajectoryLoader.load(new File("trajectory/Test.yaml"));
        }catch (Exception e){
            telemetry.addData("Error", e.toString());
            telemetry.update();
        }*/
        File trajectoryFile;

        try{
            trajectoryFile = new File("Test.yaml");
        }catch (Exception e){
            telemetry.addData("Error", e.toString());
            telemetry.update();
        }

        trajectory = new TrajectoryBuilder()

        telemetry.addData("Step 1", "init finished");
        telemetry.update();

        waitForStart();

        RamseteFollower follower = new RamseteFollower(2.0, 0.7);

        follower.followTrajectory(trajectory);


        while(opModeIsActive() && follower.isFollowing()){
            robot.drive.setDriveSignal(follower.update(robot.drive.getPoseEstimate()));
        }



    }


}

