
package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.vuforia.CameraDevice;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.android.AndroidGyroscope;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;
import java.util.ArrayList;

// autonomous program that drives bot forward a set distance, stops then
// backs up to the starting point using encoders to measure the distance.
// This example assumes there is one encoder, attached to the left motor

@Autonomous(name="Autonomous-LeftPark")
//@disabled
public class AutonomousLeftPark extends LinearOpMode
{
    RobotMecanum robotMecanum;

    @Override
    public void runOpMode() throws InterruptedException
    {
        robotMecanum = new RobotMecanum(hardwareMap, this, false);

        //==========================================================================================
        //Pre init

        robotMecanum.hooks(true);
        robotMecanum.claw(true);

        telemetry.addData("init finished", "");
        telemetry.update();

        waitForStart();

        //==========================================================================================
        //Official Start

        robotMecanum.omniTime(0.7, 0, 500, true);

        robotMecanum.driveIncrement(0.75, 0.3, 3000);

    }

    /*public void driveIncrement(double power, double increment, long totalTime)
    {

        long time = (int)(totalTime/increment);
        for(double i = 0.2; i < power; i += increment)
        {
            if(power % increment != 0 && !(i + increment < power))
                robotMecanum.omniTime(power, 0, time,true );
            else
                robotMecanum.omniTime(i + increment, 0, time, false );
        }
    }*/
}

