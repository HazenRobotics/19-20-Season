package org.firstinspires.ftc.teamcode;

//Import Code Libraries
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

//Define Code Name for phon
@TeleOp(name="TeleOpMecanum")
public class TeleopMecanum extends OpMode
{



    DcMotor lift;
    final double MAX_LIFT_SPEED = 0.8;

    RobotMecanum robotMecanum;

    //Runs once on init
    @Override
    public void init()
    {

        robotMecanum = new RobotMecanum(hardwareMap, this);

    }

    //Continues to loop
    @Override
    public void loop()
    {
        telemetry.addData("           Controls", "   ");
        telemetry.addData("Steering", "Gp1: left stick y (axis) = drive");
        telemetry.addData("Steering", "Gp1: left stick x (axis) = strafe");
        telemetry.addData("Steering", "Gp1: right stick x (axis) = rotate");
        telemetry.addData("Claw", "Gp2: b = home");
        telemetry.addData("Claw", "Gp2: x = extended");
        telemetry.addData("Lift", "Gp2: left stick y (axis)");
        telemetry.addData("Hooks", "Gp2: y = home");
        telemetry.addData("Hooks", "Gp2: a = extended    \n");

        //Claw
        if(gamepad2.b)
        {
            robotMecanum.claw(true);
        }
        if(gamepad2.x)
        {
            robotMecanum.claw(false);
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
        robotMecanum.moveOmni(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

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
