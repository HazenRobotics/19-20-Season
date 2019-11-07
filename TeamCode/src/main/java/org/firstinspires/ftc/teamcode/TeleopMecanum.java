package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name="TeleOpMecanum")
public class TeleopMecanum extends OpMode
{

    @Override
    public void init()
    {

    }

    @Override
    public void loop()
    {
        telemetry.addData("Left Stick X", gamepad1.left_stick_x);
        telemetry.addData("Left Stick Y", -gamepad1.left_stick_y);
        telemetry.addData("Right Stick X", gamepad1.right_stick_x);
        telemetry.addData("Right Stick Y", -gamepad1.right_stick_y);
        telemetry.update();
    }
}
