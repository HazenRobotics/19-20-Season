
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

    //======================================================
    HardwareMap hardwareMap;
    OpMode opMode;
    //LinearOpMode opMode;
    Telemetry telemetry;

    TensorFlow tensorFlow;

    //==============================================================================================   Robot method
    public RobotMecanum(HardwareMap hMap, OpMode opMode)
    {
        super(hMap, opMode);
    }
    public void strafe()
    {

    }
}

