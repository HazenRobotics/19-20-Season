
package org.firstinspires.ftc.teamcode;

import android.app.SharedElementCallback;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.android.AndroidGyroscope;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;
import java.util.ArrayList;

public class RobotMecanum extends Robot
{
    //Define Wheel Motors
    DcMotor frontLeftWheel;
    DcMotor backLeftWheel;
    DcMotor frontRightWheel;
    DcMotor backRightWheel;

    Servo claw;
    final double CLAW_HOME = 0.0;
    final double CLAW_EXTENDED = 0.38;
    double clawPosition = CLAW_HOME;

    //=======================================================
    HardwareMap hardwareMap;
    OpMode opMode;
    //LinearOpMode opMode;
    Telemetry telemetry;

    TensorFlow tensorFlow;

    //==============================================================================================   Robot method
    public RobotMecanum(HardwareMap hMap, OpMode opMode)
    {
        super(hMap, opMode);

        //Claw
        claw = hardwareMap.servo.get("claw");
        claw.setPosition(CLAW_HOME);
    }
    public void moveOmni(double left_stick_x, double left_stick_y, double right_stick_x)
    {
        double drive = Math.signum(-left_stick_y) * Math.pow(left_stick_y, 2);
        double strafe = Math.signum(left_stick_x) * Math.pow(left_stick_x, 2);
        double rotate = right_stick_x;

        double frontLeftPower = drive + strafe + rotate;
        double backLeftPower = -drive - strafe + rotate;
        double frontRightPower = drive - strafe - rotate;
        double backRightPower = -drive + strafe - rotate;

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
    }
    public void moveHorizontal()
    {

    }
    public void moveVertical()
    {

    }
    public void rotate()
    {

    }
    public void claw(boolean clawHome)
    {
        if (clawHome)
        {
            clawPosition = CLAW_HOME;
        }
        else
        {
            clawPosition = CLAW_EXTENDED;
        }

        claw.setPosition(clawPosition);

        telemetry.addData("Right Hook Position: ", rightHook.getPosition());
    }

}

