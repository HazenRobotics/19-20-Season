
package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.android.AndroidGyroscope;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;
import java.util.ArrayList;

//change this comment to update GitHub code

// autonomous program that drives bot forward a set distance, stops then
// backs up to the starting point using encoders to measure the distance.
// This example assumes there is one encoder, attached to the left motor

@Autonomous(name="Autonomous-TensorFlowTest")
//@disabled
public class AutonomousTensorFlowTest extends LinearOpMode
{
    @Override
    public void runOpMode() throws InterruptedException
    {
        //==========================================================================================
        //Pre init

        Robot robot = new Robot(hardwareMap, this);

        telemetry.addData("init", "finished");
        telemetry.update();

        waitForStart();

        robot.hooks(true);
        robot.clapper(true);
        sleep(100);

        //==========================================================================================
        //Official Start

        /*
        robot.turnOnSpot(90,1, true);
        sleep(250);
        */


        //robot.move(18, 1, true);
        //sleep(250);
        //robot.turnOnSpot(45,0.75, false);

        robot.tensorFlowDrive();


    }
}

