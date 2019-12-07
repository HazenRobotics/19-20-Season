
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

@Autonomous(name="AutonomousTest")
//@disabled
public class AutonomousTest extends LinearOpMode
{
    RobotMecanum robotMecanum;

    @Override
    public void runOpMode() throws InterruptedException
    {
        robotMecanum = new RobotMecanum(hardwareMap, this);

        robotMecanum.initiateVuforia();

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

        //step one, move forward to scan blocks
        //robotMecanum.drive(14, 0.75);

        /*
        //scan blocks ands organize information
        robotMecanum.tensorFlowDrive();

        telemetry.addData("Skystone Location", robotMecanum.skystoneLocation);
        telemetry.update();
        sleep(5000);
        */

        /*
        telemetry.addData("running tensorFlowDrive", "");
        telemetry.update();

        //tensor flow "drive"
        tensorFlowDrive();

        telemetry.addData("tensorFlowDrive finished", "");
        telemetry.update();

        //return back to center
        skystoneReturn();

        robotMecanum.strafeRange(14.5, 0.75, true);

        */

        //parking(true, false, 3);

/*

        robotMecanum.drive(20, 0.75);
        robotMecanum.drive(-20, 0.75);
        */

        //robotMecanum.strafeRange(10, 0.75, true);
        robotMecanum.capper(false);
        robotMecanum.capper(true);
        sleep(2000);

        robotMecanum.driveRange(30, 0.5);
        robotMecanum.hooks(false);
        robotMecanum.driveRange(1, -0.5);

        telemetry.addData("step", "finished ");
        telemetry.update();

        //robotMecanum.turnGyro(90,0.2,true);


    }

    //==============================================================================================
    /**
     *
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

