
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

@Autonomous(name="AutonomousDriverTest")
//@disabled
public class AutonomousTest extends LinearOpMode
{

    @Override
    public void runOpMode() throws InterruptedException
    {
        RobotMecanum robotMecanum = new RobotMecanum(hardwareMap, this);
        //TensorFlow tensorflow = new TensorFlow(hardwareMap, this);

        //tensorflow.initVuforia();

        //robotMecanum.gyro.calibrate();

        //==========================================================================================
        //Pre init

        //tensorflow.initVuforia();

        //robotMecanum.hooks(true);
        //robotMecanum.claw(true);

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


        robotMecanum.strafeRange(12, 0.7);

        /*
        robot.move(30, 1, false);
        sleep(250);

        leftClapper.setPosition(0.5);

        robot.turn(90,1,false,true);
        sleep(250);

        robot.clapper(true);
        robot.move(4, 1, false);
        sleep(250);

        robot.hooks(false);
        sleep(500);

        robot.move(6, 1, true);
        sleep(500);

        robot.turnGyro(45,0.75,false);
        sleep(250);

        robot.move(25, 1, false);
        sleep(250);

        robot.hooks(true);
        sleep(250);

        robot.move(2, 1, true);
        sleep(250);

        robot.turn(135,1,true,true);
        sleep(250);

        robot.move(16, 1, true);
        sleep(250);

        //turn backwards left
        robot.turn(90,1,false,true);
        sleep(250);

        robot.move(20, 1, false);
        sleep(250);

        robot.move(78, 1, true);
        sleep(250);

        robot.turnOnSpot(90,1,true);
        sleep(250);

        robot.move(6, 1, false);
        sleep(250);



        robot.hooks(false);
        sleep(500);

        tensorflow.tensorFlow();
        sleep(250);

        robot.tensorFlowDrive();
        */



    }


    //==============================================================================================



}

