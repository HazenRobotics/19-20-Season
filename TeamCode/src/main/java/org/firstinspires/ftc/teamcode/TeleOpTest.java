package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="TeleOpDriverTest")
public class TeleOpTest extends OpMode
{
    DcMotor leftMotor;
    DcMotor rightMotor;
    double leftWheelPower;
    double rightWheelPower;

    DcMotor lift;
    final double MAX_LIFT_SPEED = 0.8;

    Servo leftHook;
    Servo rightHook;
    final double LEFT_HOOK_HOME = 0.6;
    final double RIGHT_HOOK_HOME = 0.2;
    final double LEFT_HOOK_EXTENDED = 0.075;
    final double RIGHT_HOOK_EXTENDED = 0.9;
    double leftHookPosition = LEFT_HOOK_HOME;
    double rightHookPosition =  RIGHT_HOOK_HOME;
    boolean hookExtended;
    boolean hookLocked;

    Servo leftClapper;
    Servo rightClapper;
    final double LEFT_CLAPPER_HOME = 0.0;
    final double RIGHT_CLAPPER_HOME = 1.0;
    final double LEFT_CLAPPER_EXTENDED = 0.38;
    final double RIGHT_CLAPPER_EXTENDED = 0.61;
    double leftClapperPosition = LEFT_CLAPPER_HOME;
    double rightClapperPosition = RIGHT_CLAPPER_HOME;
    boolean clapperExtended;
    boolean clapperLocked;

    @Override
    public void init()
    {
        //Wheels
        leftMotor = hardwareMap.dcMotor.get("left_wheel");
        rightMotor = hardwareMap.dcMotor.get("right_wheel");
        leftMotor.setDirection(DcMotor.Direction.REVERSE);

        //Lift
        lift = hardwareMap.dcMotor.get("lift");


        //Hook
        leftHook = hardwareMap.servo.get("left_hook");
        rightHook = hardwareMap.servo.get("right_hook");
        leftHook.setPosition(LEFT_HOOK_HOME);
        rightHook.setPosition(RIGHT_HOOK_HOME);

        //Clapper
        leftClapper = hardwareMap.servo.get("left_clapper");
        rightClapper = hardwareMap.servo.get("right_clapper");
        leftClapper.setPosition(LEFT_CLAPPER_HOME);
        rightClapper.setPosition(RIGHT_CLAPPER_HOME);

    }

    @Override
    public void loop()
    {

        telemetry.addData("           Controls", "   ");
        telemetry.addData("Left Wheel", "Gp1: left stick y (axis)");
        telemetry.addData("Right Wheel", "Gp1: right stick y (axis)");
        //telemetry.addData("Grabber", "Gp1: a = home");
        //telemetry.addData("Grabber", "Gp1: y = extended");
        telemetry.addData("Clapper", "Gp2: b = home");
        telemetry.addData("Clapper", "Gp2: x = extended");
        telemetry.addData("Lift", "Gp2: left stick y (axis)");
        telemetry.addData("Hooks", "Gp2: y = home");
        telemetry.addData("Hooks", "Gp2: a = extended");
        telemetry.addData("", "");



        //||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
        //clapper

        if (gamepad2.b)
        {
            leftClapperPosition = LEFT_CLAPPER_HOME;
            rightClapperPosition = RIGHT_CLAPPER_HOME;
        }
        else if (gamepad2.x)
        {
            leftClapperPosition = LEFT_CLAPPER_EXTENDED;
            rightClapperPosition = RIGHT_CLAPPER_EXTENDED;
        }

        leftClapper.setPosition(leftClapperPosition);
        rightClapper.setPosition(rightClapperPosition);



        /*
        if (gamepad2.dpad_up)
        {
            if (leftClapperPosition <= 0.99)
                leftClapperPosition += 0.01;
            if (rightClapperPosition >= 0.01)
                rightClapperPosition -= 0.01;
        }
        else if (gamepad2.dpad_down)
        {
            if (rightClapperPosition <= 0.99)
                rightClapperPosition += 0.01;
            if (leftClapperPosition >= 0.01)
                leftClapperPosition -= 0.01;
        }


        if (gamepad2.a && !clapperLocked && !clapperExtended)
        {

            leftClapperPosition = LEFT_CLAPPER_EXTENDED;
            rightClapperPosition = RIGHT_CLAPPER_EXTENDED;

            clapperLocked = true;
            clapperExtended = true;

        }
        else if(gamepad2.a && !clapperLocked && clapperExtended)
        {

            leftClapperPosition = LEFT_CLAPPER_HOME;
            rightClapperPosition = RIGHT_CLAPPER_HOME;

            clapperLocked = true;
            clapperExtended = false;

        }
        else if(!gamepad2.a && clapperLocked)
        {

            clapperLocked = false;
        }

        leftClapper.setPosition(leftClapperPosition);
        rightClapper.setPosition(rightClapperPosition);

        */

        telemetry.addData("Left Clapper Position: ", leftClapper.getPosition());
        telemetry.addData("Right Clapper Position: ", rightClapper.getPosition());


        //||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
        //driving

        leftWheelPower = gamepad1.left_stick_y;
        rightWheelPower = gamepad1.right_stick_y;

        leftMotor.setPower(leftWheelPower);
        rightMotor.setPower(rightWheelPower);

        telemetry.addData("Right Power", rightWheelPower);
        telemetry.addData("Left Power", leftWheelPower);

        //||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
        //Lift

        /*
        if (gamepad1.left_trigger == 0 && gamepad1.right_trigger == 0)
        {
            lift.setPower(0);
        }
        else if (gamepad1.left_trigger > 0)
        {
            lift.setDirection(DcMotor.Direction.REVERSE);
            lift.setPower(gamepad1.left_trigger * MAX_LIFT_SPEED);
            telemetry.addData("Lift","up");
        }
        else if (gamepad1.right_trigger > 0)
        {
            lift.setDirection(DcMotor.Direction.FORWARD);
            lift.setPower(gamepad1.right_trigger * MAX_LIFT_SPEED * 0.4);
            telemetry.addData("Lift","down");
        }
        */

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

        //||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
        //Servo Hooks

        /*
        if (gamepad1.dpad_up)
        {
            if (leftHookPosition <= 0.99)
                leftHookPosition += 0.01;
            if (rightHookPosition >= 0.01)
                rightHookPosition -= 0.01;
        }
        else if (gamepad1.dpad_down)
        {
            if (rightHookPosition <= 0.99)
                rightHookPosition += 0.01;
            if (leftHookPosition >= 0.01)
                leftHookPosition -= 0.01;
        }

         */
        if (gamepad2.y)
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


        /*
        if (gamepad2.b && !hookLocked && !hookExtended)
        {

            leftHookPosition = LEFT_HOOK_EXTENDED;
            rightHookPosition = RIGHT_HOOK_EXTENDED;

            hookLocked = true;
            hookExtended = true;

        }
        else if(gamepad2.b && !hookLocked && hookExtended )
        {

            leftHookPosition = LEFT_HOOK_HOME;
            rightHookPosition = RIGHT_HOOK_HOME;

            hookLocked = true;
            hookExtended = false;

        }
        else if(!gamepad2.b && hookLocked)
        {

            hookLocked = false;
        }

        leftHook.setPosition(leftHookPosition);
        rightHook.setPosition(rightHookPosition);

        */
        telemetry.addData("Left Hook Position: ", leftHook.getPosition());
        telemetry.addData("Right Hook Position: ", rightHook.getPosition());

        //||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||

        telemetry.update();

    }
}
