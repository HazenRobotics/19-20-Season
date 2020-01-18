package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="ArduinoLineFollower")
//@disabled
public class ArduinoLineFollower extends LinearOpMode
{
    RobotArduino robotArduino;

    //======================================================
    int loopTest = 0;
    final double POWER = 0.6;

    @Override
    public void runOpMode() throws InterruptedException
    {
        robotArduino = new RobotArduino(hardwareMap, this);

        //==========================================================================================
        //Pre init

        telemetry.addData("init finished", "press play");
        telemetry.update();

        waitForStart();

        //==========================================================================================
        //Official Start



        while( opModeIsActive() )
        {
            robotArduino.leftWheel.setPower(POWER);
            robotArduino.rightWheel.setPower(POWER);

            robotArduino.turningMovements();

            telemetry.addData( "loop test", loopTest++  );

            telemetry.update();
        }

    }
}

