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
    final double POWER = 0.5;

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
            //robotArduino.turningMovements(POWER);

            if(robotArduino.needToTurnLeft())
                robotArduino.turnTime(POWER, 100, true);
            else if(!robotArduino.needToTurnLeft())
                robotArduino.turnTime(POWER, 100, true);

            if(  !robotArduino.colorSensorIsBlack( robotArduino.colorSensorLeft )  &&  robotArduino.colorSensorIsBlack( robotArduino.colorSensorMiddle )  &&  !robotArduino.colorSensorIsBlack( robotArduino.colorSensorRight ) )
            {
                robotArduino.leftWheel.setPower(POWER);
                robotArduino.rightWheel.setPower(POWER);
            }

            telemetry.addData( "colorSensorLeft " , robotArduino.colorSensorIsBlack( robotArduino.colorSensorLeft ) );
            telemetry.addData( "colorSensorLeft " , robotArduino.colorSensorIsBlack( robotArduino.colorSensorMiddle ) );
            telemetry.addData( "colorSensorLeft " , robotArduino.colorSensorIsBlack( robotArduino.colorSensorRight ) );
            telemetry.addData( "loop " + loopTest++, "complete" );
            telemetry.update();
        }

    }
}

