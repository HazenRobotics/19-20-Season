package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="TeleOp-MaxW")
public class RobotLongClaw extends OpMode
{
    HardwareMap hardwareMap;

    DcMotor arm;
    double armPow = 0;

    DcMotor rightMotor;
    DcMotor leftMotor;
    double leftWheelPower = 0;
    double rightWheelPower = 0;


    Servo claw;
    double clawPos = 0;

    @Override
    public void init()
    {
        //Wheels
        leftMotor = hardwareMap.dcMotor.get("left_wheel");
        rightMotor = hardwareMap.dcMotor.get("right_wheel");
        rightMotor.setDirection(DcMotor.Direction.REVERSE); //bumper

        //Arm
        arm = hardwareMap.dcMotor.get("map");

        //Claw
        claw = hardwareMap.servo.get("claw");

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



        if(gamepad1.left_bumper)
            clawPos -= 0.05;
        else if (gamepad1.right_bumper)
            clawPos += 0.05;
        claw.setPosition(clawPos);

        telemetry.addData("Arm Pos", clawPos);


        armPow = gamepad1.left_trigger;
        /*if(gamepad1.left_trigger >)
            clawPos -= 0.05;
        else if (gamepad1.right_bumper)
            clawPos += 0.05;*/
        arm.setPower(armPow);




        telemetry.addData("Arm Pos", clawPos);


        //||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
        //clapper

        telemetry.update();


    }









}
