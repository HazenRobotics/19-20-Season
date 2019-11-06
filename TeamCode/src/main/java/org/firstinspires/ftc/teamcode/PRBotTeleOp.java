package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="PRBotTeleOpTest")
public class PRBotTeleOp extends OpMode
{
    DcMotor leftMotor;
    DcMotor rightMotor;
    double leftWheelPower;
    double rightWheelPower;

    /*

    Servo leftHook;
    Servo rightHook;
    final double LEFT_HOOK_HOME = 0.5;
    final double RIGHT_HOOK_HOME = 0.5;
    final double LEFT_HOOK_EXTENDED = 1;
    final double RIGHT_HOOK_EXTENDED = 0;
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

     */

    @Override
    public void init()
    {
        //Wheels
        leftMotor = hardwareMap.dcMotor.get("left_wheel");
        rightMotor = hardwareMap.dcMotor.get("right_wheel");
        rightMotor.setDirection(DcMotor.Direction.REVERSE);
        /*

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

         */

    }

    @Override
    public void loop() {

        telemetry.addData("           Controls", "   ");
        telemetry.addData("Left Wheel", "Gp1: left stick y (axis)");
        telemetry.addData("Right Wheel", "Gp1: right stick y (axis)");


        //||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
        //driving

        leftWheelPower = gamepad1.left_stick_y;
        rightWheelPower = gamepad1.right_stick_y;

        leftMotor.setPower(leftWheelPower);
        rightMotor.setPower(rightWheelPower);

        telemetry.addData("Right Power", rightWheelPower);
        telemetry.addData("Left Power", leftWheelPower);


        /*
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




        telemetry.addData("Left Clapper Position: ", leftClapper.getPosition());
        telemetry.addData("Right Clapper Position: ", rightClapper.getPosition());

*/



/*
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


        if (gamepad2.y)
        {
            leftHookPosition = LEFT_HOOK_HOME;
            rightHookPosition = RIGHT_HOOK_HOME;
        } else if (gamepad2.a)
        {
            leftHookPosition = LEFT_HOOK_EXTENDED;
            rightHookPosition = RIGHT_HOOK_EXTENDED;
        }

        leftHook.setPosition(leftHookPosition);
        rightHook.setPosition(rightHookPosition);

*/
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
        /*
        telemetry.addData("Left Hook Position: ", leftHook.getPosition());
        telemetry.addData("Right Hook Position: ", rightHook.getPosition());

        //||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||

        telemetry.update();

        */
    }
}
