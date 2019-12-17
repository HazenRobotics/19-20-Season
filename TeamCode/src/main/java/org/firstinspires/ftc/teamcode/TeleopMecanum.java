package org.firstinspires.ftc.teamcode;

//Import Code Libraries
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

//Define Code Name for phon
@TeleOp(name="TeleOpMecanum")
public class TeleopMecanum extends OpMode
{
    DcMotor lift;
    final double MAX_LIFT_SPEED = 1;

    RobotMecanum robotMecanum;

    //Runs once on init
    @Override
    public void init()
    {
        robotMecanum = new RobotMecanum(hardwareMap, this, false);
        robotMecanum.telemetry.setAutoClear(true);
    }


    //Continues to loop
    @Override
    public void loop()
    {
        /*telemetry.addData("           Controls", "   ");
        telemetry.addData("Steering", "Gp1: left stick y (axis) = drive");
        telemetry.addData("Steering", "Gp1: left stick x (axis) = strafe");
        telemetry.addData("Steering", "Gp1: right stick x (axis) = rotate");
        telemetry.addData("Claw", "Gp2: b = home");
        telemetry.addData("Claw", "Gp2: x = extended");
        telemetry.addData("Lift", "Gp2: left stick y (axis)");
        telemetry.addData("Hooks", "Gp2: y = home");
        telemetry.addData("Hooks", "Gp2: a = extended    \n");
        robotMecanum.printGyroHeading();*/

        telemetry.addData("front right sensor", robotMecanum.rangeSensorRightFront.getDistance(DistanceUnit.INCH));
        telemetry.addData("back right sensor", robotMecanum.rangeSensorRightBack.getDistance(DistanceUnit.INCH));
        telemetry.addData("front left sensor", robotMecanum.rangeSensorLeftFront.getDistance(DistanceUnit.INCH));
        telemetry.addData("back left sensor", robotMecanum.rangeSensorLeftBack.getDistance(DistanceUnit.INCH));

        //Claw
        if(gamepad2.b)
        {
            robotMecanum.claw(true);
        }
        if(gamepad2.x)
        {
            robotMecanum.claw(false);
        }

        //Claw
        if(!(gamepad2.right_trigger == 0))
        {
            robotMecanum.capper(true);
        }
        else
        {
            robotMecanum.capper(false);
        }

        //Servo Hooks
        if (gamepad2.y)
        {
            robotMecanum.hooks(true);
        }
        else if (gamepad2.a)
        {
            robotMecanum.hooks(false);
        }

        //Driving
        robotMecanum.moveOmni(-gamepad1.left_stick_y, gamepad1.right_stick_x, -gamepad1.left_stick_x);

        //robotMecanum.rightHook.setPosition(gamepad1.right_trigger);
        //robotMecanum.leftHook.setPosition(gamepad1.left_trigger);

        /*
        //Lift
        if (gamepad2.left_stick_y > 0)
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
        {
            lift.setPower(0);
        }

         */
        robotMecanum.lift.setPower(gamepad2.left_stick_y * MAX_LIFT_SPEED);


        telemetry.update();
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
