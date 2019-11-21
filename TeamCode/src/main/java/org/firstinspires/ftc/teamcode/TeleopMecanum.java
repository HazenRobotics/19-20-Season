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
    RobotMecanum robotMecanum ;
    /*
    //Define Wheel Motors
    DcMotor frontLeftWheel;
    DcMotor backLeftWheel;
    DcMotor frontRightWheel;
    DcMotor backRightWheel;

    //Variables to store wheel power
    double frontLeftPower;
    double backLeftPower;
    double frontRightPower;
    double backRightPower;

    //Variables for Mecanum input
    double drive;
    double strafe;
    double rotate;

    DcMotor lift;
    final double MAX_LIFT_SPEED = 0.8;

    //Hooks
    Servo leftHook;
    Servo rightHook;
    final double LEFT_HOOK_HOME = 0.6;
    final double RIGHT_HOOK_HOME = 0.2;
    final double LEFT_HOOK_EXTENDED = 0.075;
    final double RIGHT_HOOK_EXTENDED = 0.9;
    double leftHookPosition = LEFT_HOOK_HOME;
    double rightHookPosition =  RIGHT_HOOK_HOME;

    //Claw
    Servo claw;
    final double CLAW_HOME = 0.0;
    final double CLAW_EXTENDED = 0.38;
    double clawPosition = CLAW_HOME;
    */

    //Runs once on init
    @Override
    public void init()
    {
        robotMecanum = new RobotMecanum(hardwareMap, this);
        /*
        //Get hardware map definitions for the
        frontLeftWheel = hardwareMap.dcMotor.get("front_left_wheel");
        backLeftWheel = hardwareMap.dcMotor.get("back_left_wheel");
        frontRightWheel = hardwareMap.dcMotor.get("front_right_wheel");
        backRightWheel = hardwareMap.dcMotor.get("back_right_wheel");

        //Reverse the two flipped wheels
        frontRightWheel.setDirection(DcMotor.Direction.REVERSE);
        backRightWheel.setDirection(DcMotor.Direction.REVERSE);

        //Lift
        lift = hardwareMap.dcMotor.get("lift");

        //Hook
        leftHook = hardwareMap.servo.get("left_hook");
        rightHook = hardwareMap.servo.get("right_hook");
        leftHook.setPosition(LEFT_HOOK_HOME);
        rightHook.setPosition(RIGHT_HOOK_HOME);

        //Claw
        claw = hardwareMap.servo.get("claw");
        claw.setPosition(CLAW_HOME);

         */
    }

    //Continues to loop
    @Override
    public void loop()
    {
        telemetry.addData("           Controls", "   ");
        telemetry.addData("??-Steering-??", "??-Gp1: left stick y (axis)-??");
        telemetry.addData("??-Steering-??", "??-Gp1: right stick y (axis)-??");
        telemetry.addData("Claw", "Gp2: b = home");
        telemetry.addData("Claw", "Gp2: x = extended");
        telemetry.addData("Lift", "Gp2: left stick y (axis)");
        //telemetry.addData("Hooks", "Gp2: y = home");
        //telemetry.addData("Hooks", "Gp2: a = extended");
        telemetry.addData("", "");



        //||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
        //Claw


        /*
        if(gamepad2.y)
        {
            robotMecanum.claw(true);
        }
        if(gamepad2.x)
        {
            robotMecanum.claw(false);
        }



        //||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
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
        }*/

        //||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
        //Servo Hooks
        /*if (gamepad2.y)
        {
            leftHookPosition = LEFT_HOOK_HOME;
            rightHookPosition = RIGHT_HOOK_HOME;
        }
        else if (gamepad2.a)
        {
            leftHookPosition = LEFT_HOOK_EXTENDED;
            rightHookPosition = RIGHT_HOOK_EXTENDED;
        }

        leftHook.setPosition(leftHookPosition);
        rightHook.setPosition(rightHookPosition);

        telemetry.addData("Left Hook Position: ", leftHook.getPosition());
        telemetry.addData("Right Hook Position: ", rightHook.getPosition());
         */

        //||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
        //driving
        /*
        //Convert gamepad data into custom variables
        drive = Math.signum(-gamepad1.left_stick_y) * Math.pow(gamepad1.left_stick_y, 2);
        strafe = Math.signum(gamepad1.left_stick_x) * Math.pow(gamepad1.left_stick_x, 2);
        rotate = gamepad1.right_stick_x;

        //Set wheel power according to input
        frontLeftPower = drive + strafe + rotate;
        backLeftPower = -drive - strafe + rotate;
        frontRightPower = drive - strafe - rotate;
        backRightPower = -drive + strafe - rotate;

        //Set the wheel power according to variables
        frontLeftWheel.setPower(frontLeftPower);
        backLeftWheel.setPower(backLeftPower);
        frontRightWheel.setPower(frontRightPower);
        backRightWheel.setPower(backRightPower);
        */


        robotMecanum.moveOmni(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);

        //||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||


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
