
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

@Autonomous(name="Autonomous-RightPark")
//@disabled
public class AutonomousRightPark extends LinearOpMode
{
    //==============================================================================================

    @Override
    public void runOpMode() throws InterruptedException
    {
        RobotMecanum robotMecanum = new RobotMecanum(hardwareMap, this, false);

        //==========================================================================================
        //Pre init

        telemetry.addData("init finished", "");
        telemetry.update();

        waitForStart();

        robotMecanum.frontLeftWheel.setPower(0.75);
        sleep(1000);
        robotMecanum.frontLeftWheel.setPower(0);
        sleep(2000);

        robotMecanum.frontRightWheel.setPower(0.75);
        sleep(1000);
        robotMecanum.frontRightWheel.setPower(0);
        sleep(2000);

        robotMecanum.backLeftWheel.setPower(0.75);
        sleep(1000);
        robotMecanum.backLeftWheel.setPower(0);
        sleep(2000);

        robotMecanum.backRightWheel.setPower(0.75);
        sleep(1000);
        robotMecanum.backRightWheel.setPower(0);

        //==========================================================================================
        //Official Start

    }
}

