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
            robotArduino.move(5, 0.6, true);
            robotArduino.turnOnSpot(180, 0.6, true);
            robotArduino.move(10, 0.6, true);
            robotArduino.turn(10, 0.6, true, false);
            break;
        }

        /*
        while( opModeIsActive() )
        {
            robotArduino.move(1, 0.6, true);

            robotArduino.turningMovements();

            telemetry.addData( "loop test", loopTest++  );

            telemetry.update();
        }
         */
    }
}

