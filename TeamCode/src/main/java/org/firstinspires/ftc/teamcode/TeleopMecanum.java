package org.firstinspires.ftc.teamcode;

//Import Code Libraries
import android.sax.TextElementListener;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import static java.lang.Thread.sleep;


//Define Code Name for phon
@TeleOp(name="TeleOpMecanum")
public class TeleopMecanum extends OpMode
{

    final double MAX_LIFT_SPEED = 0.9;
    int sensitivityCount = 2;

    RobotMecanum robotMecanum;

    //Runs once on init
    @Override
    public void init()
    {
        robotMecanum = new RobotMecanum(hardwareMap, this, true);
        telemetry.addData("robotMecanum setup", "finished");
        telemetry.update();
        //sleep(2000);
        robotMecanum.sleepRobot(2000);

        robotMecanum.telemetry.setAutoClear(true);
        /*Thread ledThread = new LedThread();
        ledThread.start();*/
        telemetry.addData("init", "finished");
        telemetry.update();
    }


    //Continues to loop
    @Override
    public void loop()
    {

        /*
        telemetry.addData("           Controls", "   ");
        telemetry.addData("Steering", "Gp1: left stick y (axis) = drive");
        telemetry.addData("Steering", "Gp1: left stick x (axis) = strafe");
        telemetry.addData("Steering", "Gp1: right stick x (axis) = rotate");
        telemetry.addData("Claw", "Gp2: b = home");
        telemetry.addData("Claw", "Gp2: x = extended");
        telemetry.addData("Lift", "Gp2: left stick y (axis)");
        telemetry.addData("Hooks", "Gp2: y = home");
        telemetry.addData("Hooks", "Gp2: a = extended    \n");*/

        /*telemetry.addData("Old Gyro  ", robotMecanum.gyro.getHeading());
        telemetry.addData("New Gyro  ", robotMecanum.getNewGyroHeading());*/
        telemetry.addData("Lift      ", robotMecanum.lift.getCurrentPosition());
        telemetry.addData("front left wheel", robotMecanum.frontLeftWheel.getCurrentPosition());
        telemetry.addData("back left wheel", robotMecanum.backLeftWheel.getCurrentPosition());
        telemetry.addData("front right wheel", robotMecanum.frontRightWheel.getCurrentPosition());
        telemetry.addData("back right wheel", robotMecanum.backRightWheel.getCurrentPosition());

        /*
        telemetry.addData("Right front", robotMecanum.rangeSensorRightFront.getDistance(DistanceUnit.INCH));
        telemetry.addData("Right back", robotMecanum.rangeSensorRightBack.getDistance(DistanceUnit.INCH));
        telemetry.addData("Left front", robotMecanum.rangeSensorLeftFront.getDistance(DistanceUnit.INCH));
        telemetry.addData("Left back", robotMecanum.rangeSensorLeftBack.getDistance(DistanceUnit.INCH));
        telemetry.addData("Back right", robotMecanum.rangeSensorBackRight.getDistance(DistanceUnit.INCH));
        telemetry.addData("Back left", robotMecanum.rangeSensorBackLeft.getDistance(DistanceUnit.INCH));
        */


        //Lift
        robotMecanum.lift.setPower(-gamepad2.left_stick_y * MAX_LIFT_SPEED);

        //Claw
        if(gamepad2.b)
        {
            robotMecanum.claw(true);
        }
        if(gamepad2.x)
        {
            robotMecanum.claw(false);
        }

        //Clapper
        /*if(!(gamepad2.right_trigger == 0))
        {
            robotMecanum.capper(true);
        }
        else
        {
            robotMecanum.capper(false);
        }*/

        //Servo Hooks
        if (gamepad2.y)
        {
            robotMecanum.hooks(true);
        }
        else if (gamepad2.a)
        {
            robotMecanum.hooks(false);
        }
        
        sensitivityCount -= gamepad1.left_bumper && sensitivityCount >= 3 ? 1 : 0;
        sensitivityCount += gamepad1.right_bumper && sensitivityCount <= 5 ? 1 : 0;

        //Driving
        robotMecanum.moveOmniAdjustable(-gamepad1.left_stick_y, gamepad1.right_stick_x, gamepad1.left_stick_x, sensitivityCount);

        //robotMecanum.rightHook.setPosition(gamepad1.right_trigger);
        //robotMecanum.leftHook.setPosition(gamepad1.left_trigger);

        /*if (gamepad2.left_stick_y > 0)
        {
            lift.setPower(gamepad2.left_stick_y * MAX_LIFT_SPEED * 0.4);
            telemetry.addData("Lift", "down");
        }
        else if (gamepad2.left_stick_y < 0)
        {
            lift.setPower(gamepad2.left_stick_y * MAX_LIFT_SPEED);
            telemetry.addData("Lift", "up");
        }
        else
            lift.setPower(0);

         */

        /*

        //front left wheel power = y
        if(gamepad1.y)
            robotMecanum.frontLeftWheel.setPower(1);
        else
            robotMecanum.frontLeftWheel.setPower(0);

        //front right wheel power = b
        if(gamepad1.b)
            robotMecanum.frontRightWheel.setPower(1);
        else
            robotMecanum.frontRightWheel.setPower(0);

        //back left wheel power = x
        if(gamepad1.x)
            robotMecanum.backLeftWheel.setPower(1);
        else
            robotMecanum.backLeftWheel.setPower(0);

        //back right wheel power = a
        if(gamepad1.a)
            robotMecanum.backRightWheel.setPower(1);
        else
            robotMecanum.backRightWheel.setPower(0);*/


        telemetry.update();
    }

    private class LedThread extends Thread
    {
        public LedThread()
        {
            this.setName("LedThread");
        }

        @Override
        public void run()
        {
            try
            {
                while(!isInterrupted())
                {
                    robotMecanum.setRGB(0,0,1);
                    sleep(2000);
                    robotMecanum.setRGB(1, 1,0);
                    sleep(2000);
                }

            }
            catch (InterruptedException e){}
            catch (Exception e){}
        }
    }
}


/**
 *
 * front left motor = # 1 - P0          front right = # 1 - P1
 * back left motor = # 2 - P0`          back right = # 2 - P1
 *
 *
 *
 *
 */
