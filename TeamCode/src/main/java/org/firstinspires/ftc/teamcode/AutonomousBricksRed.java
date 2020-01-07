
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

@Autonomous(name="Autonomous-BricksRed")
//@disabled
public class AutonomousBricksRed extends LinearOpMode
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


        //drive forward
        robotMecanum.omniTime(0.7, 0, 1000);
        sleep(500);
        //claw down/extended
        robotMecanum.claw(false);

        //drive back slightly
        robotMecanum.omniTime(-0.75, 0, 500);

        //strafe right to wall
        robotMecanum.omniTime(0, 0.75, 3600);

        //claw up/home
        robotMecanum.claw(true);

        //strafe left ot other side
        robotMecanum.omniTime(0, -0.75, 3600);

        //forward
        robotMecanum.omniTime(0.75, 0, 500);

        //grab block
        robotMecanum.claw(true);

        //back small amount
        robotMecanum.omniTime(-0.75, 0, 500);

        //strafe
        robotMecanum.omniTime(0, -0.75, 4000);

        //drop
        robotMecanum.claw(false);



    }
    public void sideFoundation(boolean isRedField, int waitTime, int strafe2Time, int drive1Time, int drive2Time)
    {

        if(isRedField)
        {
            robotMecanum.strafeRange(16, 0.75, true);

            //driveTime(0.65, 900);

            robotMecanum.omniTime(0.75, 0, drive1Time);
            //robotMecanum.driveRange(34, 0.7);
            robotMecanum.hooks(false);
            sleep(250);
            robotMecanum.omniTime(-0.75, 0, drive2Time);
            //robotMecanum.driveRange(1, -0.75);
            robotMecanum.hooks(true);

            robotMecanum.omniTime(0, 0.75, strafe2Time);
            //robotMecanum.strafeRange(55, 0.75, true);
        }
        else
        {
            robotMecanum.strafeRange(16, -0.75, false);

            //driveTime(0.65, 900);

            robotMecanum.omniTime(0.75, 0, drive1Time);
            //robotMecanum.driveRange(34, 0.7);
            robotMecanum.hooks(false);
            sleep(250);
            robotMecanum.omniTime(-0.75, 0, drive2Time);
            //robotMecanum.driveRange(1, -0.75);
            robotMecanum.hooks(true);

            robotMecanum.omniTime(0, 0.75, strafe2Time);
            //robotMecanum.strafeRange(55, 0.75, false);
        }
        sleep(waitTime);
    }
    //==============================================================================================
    /**
     * @param isParkingLeft - parking to the left of robot
     * @param isParkingFar  - parking far from starting wall
     * @param sleepTime     - length of time to wait/sleep in SECONDS
     */
    public void parking(boolean isParkingLeft, boolean isParkingFar, int sleepTime)
    {
        sleep(sleepTime * 1000);

        if(isParkingFar)
        {
            robotMecanum.drive(18, 0.75);
        }
        if(isParkingLeft)
        {
            robotMecanum.strafeRange(65, -0.75, true);
        }
        else
        {
            robotMecanum.strafeRange(65, 0.75, false);
        }
    }
    public void skystoneReturn()
    {
        robotMecanum.strafeRange(74, 0.75, false);
    }

    public void skystoneNone()
    {
        //strafe over to block
        robotMecanum.strafeRange(14, -0.75, false);

        //drive forward to pickup block
        robotMecanum.drive(12, 0.75);

        //pickup block
        robotMecanum.claw(true);

        //drive backward
        robotMecanum.drive(-12, 0.75);

        telemetry.addData("skystoneNone", "completed");
        telemetry.update();
    }

    public void skystoneLeft()
    {
        //strafe over to block
        robotMecanum.strafeRange(22, -0.75, false);

        //drive forward to pickup block
        robotMecanum.drive(12, 0.75);

        //pickup block
        robotMecanum.claw(true);

        //drive backward
        robotMecanum.drive(-12, 0.75);

        telemetry.addData("skystoneNone", "completed");
        telemetry.update();
    }

    public void skystoneRight()
    {
        //strafe over to block
        robotMecanum.strafeRange(30, -0.75, false);

        //drive forward to pickup block
        robotMecanum.drive(12, 0.75);

        //pickup block
        robotMecanum.claw(true);

        //drive backward
        robotMecanum.drive(-12, 0.75);

        telemetry.addData("skystoneNone", "completed");
        telemetry.update();
    }

    public void tensorFlowDrive()
    {
        if ( robotMecanum.skystoneLocation.equals("none") )
        {
            telemetry.addData("move to the skystone offscreen", "");
            telemetry.update();
            skystoneNone();
        }
        else if ( robotMecanum.skystoneLocation.equals("left") )
        {
            telemetry.addData("move to the left skystone position", "");
            telemetry.update();
            skystoneLeft();
        }
        else if ( robotMecanum.skystoneLocation.equals("right") )
        {
            telemetry.addData("move to the right skystone position", "");
            telemetry.update();
            skystoneRight();
        }
        else
            telemetry.addData("Error: ", "No Move");
        telemetry.update();
    }


}

