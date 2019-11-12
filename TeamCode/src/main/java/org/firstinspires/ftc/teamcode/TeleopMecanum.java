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

    //Runs once on init
    @Override
    public void init()
    {
        //Get hardware map definitions for the
        frontLeftWheel = hardwareMap.dcMotor.get("front_left_wheel");
        backLeftWheel = hardwareMap.dcMotor.get("back_left_wheel");
        frontRightWheel = hardwareMap.dcMotor.get("front_right_wheel");
        backRightWheel = hardwareMap.dcMotor.get("back_right_wheel");

        //Reverse the two flipped wheels
        frontRightWheel.setDirection(DcMotor.Direction.REVERSE);
        backRightWheel.setDirection(DcMotor.Direction.REVERSE);
    }

    //Continues to loop
    @Override
    public void loop()
    {
        //Convert gamepad data into custom variables
        drive = Math.signum(-gamepad1.left_stick_y) * Math.pow(gamepad1.left_stick_y, 4);
        strafe = Math.signum(gamepad1.left_stick_x) * Math.pow(gamepad1.left_stick_x, 4);
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

        //Print Telementary Data for the wheels
        telemetry.addData("frontLeftPower", frontLeftPower);
        telemetry.addData("backLeftPower", backLeftPower);
        telemetry.addData("frontRightPower", frontRightPower);
        telemetry.addData("backRightPower", backRightPower);
        telemetry.update();
    }
}
