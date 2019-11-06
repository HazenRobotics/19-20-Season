
package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.vuforia.CameraDevice;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.android.AndroidGyroscope;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;
import java.util.ArrayList;

//change this comment to update GitHub code

// autonomous program that drives bot forward a set distance, stops then
// backs up to the starting point using encoders to measure the distance.
// This example assumes there is one encoder, attached to the left motor

@Autonomous(name="Autonomous-BricksLeft")
//@disabled
public class AutonomousBricksLeft extends LinearOpMode
{
    Robot robot = new Robot();
    TensorFlow tensorflow = new TensorFlow();

    //======================================================
    DcMotor leftMotor;
    DcMotor rightMotor;

    //======================================================
    DcMotor lift;
    final double MAX_LIFT_SPEED = 0.8;

    //======================================================
    Servo leftHook;
    Servo rightHook;
    final double LEFT_HOOK_HOME = 0.75;
    final double RIGHT_HOOK_HOME = 0.2;
    final double LEFT_HOOK_EXTENDED = 0;
    final double RIGHT_HOOK_EXTENDED = 1;
    double leftHookPosition = LEFT_HOOK_HOME;
    double rightHookPosition =  RIGHT_HOOK_HOME;

    //======================================================
    Servo leftClapper;
    Servo rightClapper;
    final double LEFT_CLAPPER_HOME = 0.0;
    final double RIGHT_CLAPPER_HOME = 1.0;
    final double LEFT_CLAPPER_EXTENDED = 0.38;
    final double RIGHT_CLAPPER_EXTENDED = 0.61;
    double leftClapperPosition = LEFT_CLAPPER_HOME;
    double rightClapperPosition = RIGHT_CLAPPER_HOME;

    //======================================================
    GyroSensor gyro;

    //======================================================
    final int tickPerRevlolution = 1440;
    final double linearWheelDistance = (Math.PI) * 4;
    final double linearSpoolDistance = (Math.PI) * 1.5748;

    //======================================================
    enum Position{none,left,right};
    Position skystonePosition;

    int noneTally;
    int leftTally;
    int rightTally;
    int totalTally;

    final int IMAGE_CHECK_ITERATIONS = 2;

    //==============================================================
    int shuffleCount = 0;

    //==============================================================================================
    //Tensor Flow

    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Stone";
    private static final String LABEL_SECOND_ELEMENT = "Skystone";

    /* IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.   */

    private static final String VUFORIA_KEY = "ATBM8BX/////AAABmYUPteQH1kxGoIW74fdllZ4hBtjknbn0M15daiyIdyA3MB9HGtgKfQ5lm7VzJ5KNCTQULwwr+QvwOfmMgLj4zqYgNo+qr4Z68INxnaD47x5Dk2qss6ta/E7uOTEr/9fbz76ecBIHhPfdC7QPgCRSFAjIcdcXgSlcXV0kX8eO5XhB0QQ6jAuHLqWG9zeBCp0Hhdm7K6glT+Ab/0rnA8DMu4Vws6f57tnPa3OgX5pMj+D7cA3D+9Y0JxeDmOeG5EeYL8QZOtGriFeN5P7JDcfgqFkvnVOQTFfWndULWhlPVJVIaGNzhIPrFi803OWGr2bVTeYwBXIhUYXQDek2LmyPuk115vOTX19Lt+mwfmNxo0/p";

    //{@link #vuforia} is the variable we will use to store our instance of the Vuforia localization engine.
    private VuforiaLocalizer vuforia;

    //@link #tfod} is the variable we will use to store our instance of the TensorFlow Object Detection engine.
    private TFObjectDetector tfod;

    //==============================================================================================

    @Override
    public void runOpMode() throws InterruptedException
    {
        //arm = hardwareMap.servo.get("arm");

        //Map Hardware
        lift = hardwareMap.dcMotor.get("lift");

        leftMotor = hardwareMap.dcMotor.get("left_wheel");
        rightMotor = hardwareMap.dcMotor.get("right_wheel");

        leftHook = hardwareMap.servo.get("left_hook");
        rightHook = hardwareMap.servo.get("right_hook");

        leftClapper = hardwareMap.servo.get("left_clapper");
        rightClapper = hardwareMap.servo.get("right_clapper");

        gyro = hardwareMap.gyroSensor.get("gyro");

        gyro.calibrate();

        //==========================================================================================
        //Pre init

        //initVuforia();

        robot.hooks(true);
        robot.clapper(true);
        sleep(500);

        telemetry.addData("init finished", "");
        telemetry.update();

        waitForStart();


        //==========================================================================================
        //Official Start

        robot.move(33, 1, true);
        sleep(250);

        robot.clapper(false);

        robot.move(12, 1, false);
        sleep(250);

        robot.turnOnSpot(90, 1,true);
        sleep(250);



        robot.move(20, 1, true);
        sleep(250);

        robot.clapper(true);

        robot.move(34, 1, false);
        sleep(250);

        robot.turnOnSpot(90, 1, false);
        sleep(250);

        //==========================================================================================

        robot.move(12, 1, true);
        sleep(250);

        robot.clapper(false);

        robot.move(12, 1, false);
        sleep(250);



        robot.turnOnSpot(75, 1, true);
        sleep(250);

        robot.move(30, 1, true);
        sleep(250);

        robot.clapper(true);



    }
}

