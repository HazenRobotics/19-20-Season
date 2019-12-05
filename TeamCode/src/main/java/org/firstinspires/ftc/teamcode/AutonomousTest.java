
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
    TensorFlow tensorflow;

    @Override
    public void runOpMode() throws InterruptedException
    {
        robotMecanum = new RobotMecanum(hardwareMap, this);
        tensorflow = new TensorFlow(hardwareMap, this);

        //tensorflow.initVuforia();

        robotMecanum.gyro.calibrate();

        //==========================================================================================
        //Pre init

        //tensorflow.initVuforia();

        robotMecanum.hooks(true);
        robotMecanum.claw(true);

        telemetry.addData("Step 1", "init finished");
        telemetry.update();

        waitForStart();



        //==========================================================================================
        //Official Start

        /*while(opModeIsActive()){
            telemetry.addData("back right distance", robotMecanum.rangeSensorRightBack.getDistance(DistanceUnit.INCH));
            telemetry.addData("front right distance", robotMecanum.rangeSensorRightFront.getDistance(DistanceUnit.INCH));
            telemetry.addData("back left distance", robotMecanum.rangeSensorLeftBack.getDistance(DistanceUnit.INCH));
            telemetry.addData("front left distance", robotMecanum.rangeSensorLeftFront.getDistance(DistanceUnit.INCH));
            telemetry.update();
        }*/

        //step one, move forward to scan blocks
        robotMecanum.drive(18, 0.75);


        //scan blocks
        robotMecanum.tensorFlowDrive();

        tensorFlowDrive();

        //return back to center
        //skystoneReturn();

        //
        //robotMecanum.strafeRange(18, 0.75, true);



        //robotMecanum.turnGyro(90,0.2,true);



        /*
        robot.hooks(false);
        sleep(500);

        tensorflow.tensorFlow();
        sleep(250);


        */



    }


    //==============================================================================================

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
        //robotMecanum.claw(true);

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
        //robotMecanum.claw(true);

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
        //robotMecanum.claw(true);

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
            skystoneNone();
        }
        else if ( robotMecanum.skystoneLocation.equals("left") )
        {
            telemetry.addData("move to the left skystone position", "");
            skystoneLeft();
        }
        else if ( robotMecanum.skystoneLocation.equals("right") )
        {
            telemetry.addData("move to the right skystone position", "");
            skystoneRight();
        }
        else
            telemetry.addData("Error: ", "No Move");
        telemetry.update();
    }


}

