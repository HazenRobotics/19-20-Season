
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

// autonomous program that drives bot forward a set distance, stops then
// backs up to the starting point using encoders to measure the distance.
// This example assumes there is one encoder, attached to the left motor

@Autonomous(name="Autonomous-FoundationLeft")
//@disabled
public class AutonomousFoundationLeft extends LinearOpMode
{
    RobotMecanum robotMecanum;

    @Override
    public void runOpMode() throws InterruptedException
    {
        robotMecanum = new RobotMecanum(hardwareMap, this, false);

        //robotMecanum.initiateVuforia();

        robotMecanum.gyro.calibrate();

        //==========================================================================================
        //Pre init

        robotMecanum.hooks(true);
        robotMecanum.claw(true);

        telemetry.addData("Step 1", "init finished");
        telemetry.update();

        waitForStart();

        //==========================================================================================
        //Official Start

        sideFoundation(true, 20000, 3000, 3000, 3000);

        sideFoundation(true, 20000, 4000, 4000, 4000);

        sideFoundation(true, 20000, 5000, 5000, 5000);

    }
    public void sideFoundation(boolean isRedField, int waitTime, int strafe2Time, int drive1Time, int drive2Time)
{

    if(isRedField)
    {
        robotMecanum.strafeRange(16, 0.75, true);

        //driveTime(0.65, 900);

        robotMecanum.driveTime(0.75, drive1Time);
        //robotMecanum.driveRange(34, 0.7);
        robotMecanum.hooks(false);
        sleep(250);
        robotMecanum.driveTime(-0.75, drive2Time);
        //robotMecanum.driveRange(1, -0.75);
        robotMecanum.hooks(true);

        robotMecanum.strafeTime(0.75, strafe2Time);
        //robotMecanum.strafeRange(55, 0.75, true);
    }
    else
    {
        robotMecanum.strafeRange(16, -0.75, false);

        //driveTime(0.65, 900);

        robotMecanum.driveTime(0.75, drive1Time);
        //robotMecanum.driveRange(34, 0.7);
        robotMecanum.hooks(false);
        sleep(250);
        robotMecanum.driveTime(-0.75, drive2Time);
        //robotMecanum.driveRange(1, -0.75);
        robotMecanum.hooks(true);

        robotMecanum.strafeTime(0.75, strafe2Time);
        //robotMecanum.strafeRange(55, 0.75, false);
    }
    sleep(waitTime);
}
}

