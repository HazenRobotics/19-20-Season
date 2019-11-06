package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

@TeleOp(name="LightTest")
public class LightTest extends LinearOpMode {

    private DcMotor redControl;
    private DcMotor greenControl;
    private DcMotor blueControl;

    private double targetTime;

    @Override
    public void runOpMode() throws InterruptedException {
        redControl = hardwareMap.dcMotor.get("red");
        greenControl = hardwareMap.dcMotor.get("green");
        blueControl = hardwareMap.dcMotor.get("blue");

        waitForStart();

        while(opModeIsActive()) {
            setLED(1.0, 1.0, 0.0);
            targetTime = getRuntime() + 2;
            while(getRuntime() < targetTime){

            }

            setLED(0.0, 1.0, 1.0);
            targetTime = getRuntime() + 2;
            while(getRuntime() < targetTime){

            }

            setLED(1.0, 0.0, 1.0);
            targetTime = getRuntime() + 2;
            while(getRuntime() < targetTime){

            }
        }
    }
    public void setLED(double red, double green, double blue){
        redControl.setPower(red);
        greenControl.setPower(green);
        blueControl.setPower(blue);
    }
    public void setLED(double redValue, double greenValue, double blueValue, int EFFECT){

    }
}
