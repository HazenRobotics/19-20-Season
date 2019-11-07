
package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.android.AndroidGyroscope;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;
import java.util.ArrayList;

public class TensorFlow extends LinearOpMode
{

    Robot robot = new Robot();

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
    public void runOpMode() throws InterruptedException { }

    //==========================================================================================

    public void skystoneNone()
    {
        telemetry.addData("skystoneNone", "running");
        telemetry.update();
        //Position to pick up skystone

        //Dive and pick up skystone
        robot.move(12, 1, true);


        robot.clapper(false);

        //Back up with skystone and rotate
        robot.move(12, 1, false);

        //move to the preplanned position
        robot.move(12, 1, true);

        //Run method to return place skystone on foundation
        skystoneReturn();
    }

    public void skystoneLeft()
    {
        telemetry.addData("skystoneLeft", "running");
        telemetry.update();
        //Position to pick up skystone

        //Dive and pick up skystone
        robot.move(12, 1, true);

        robot.clapper(false);

        //Back up with skystone and rotate
        robot.move(12, 1, false);

        //move to the preplanned position
        //-Already there

        //Run method to return place skystone on foundation
        skystoneReturn();
    }

    public void skystoneRight()
    {
        telemetry.addData("skystoneRight", "running");
        telemetry.update();
        //Position to pick up skystone

        //Dive and pick up skystone
        robot.move(4, 1, true);
        robot.clapper(false);

        //Back up with skystone and rotate
        robot.move(6, 1, false);

        //move to the preplanned position
        robot.move(6, 1, true);

        //Run method to return place skystone on foundation
        skystoneReturn();
    }

    public void skystoneReturn()
    {
        robot.move(24, 1, true);



        robot.move(12, 1, true);
    }

    //==========================================================================================
    //hook methd

    public void hooks(boolean hooksHome)
    {

        telemetry.addData("hooks", "running");
        telemetry.update();

        //set hooks positions to positions
        if(hooksHome)
        {
            leftHookPosition = LEFT_HOOK_HOME;
            rightHookPosition = RIGHT_HOOK_HOME;
        }
        else
        {
            leftHookPosition = LEFT_HOOK_EXTENDED;
            rightHookPosition = RIGHT_HOOK_EXTENDED;
        }
        leftHook.setPosition(leftHookPosition);
        rightHook.setPosition(rightHookPosition);
    }

    //==============================================================================================
    //Tensor Flow

    public void tensorFlow()
    {

        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that first.

        sleep(2000);

        if (ClassFactory.getInstance().canCreateTFObjectDetector())
            initTfod();

        else
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");

        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod != null)
            tfod.activate();

        //Wait for the game to begin
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();

        if (opModeIsActive())
        {
            while (opModeIsActive() && (leftTally < IMAGE_CHECK_ITERATIONS && rightTally < IMAGE_CHECK_ITERATIONS && noneTally < IMAGE_CHECK_ITERATIONS) )
            {
                while (noneTally != IMAGE_CHECK_ITERATIONS && leftTally != IMAGE_CHECK_ITERATIONS && rightTally != IMAGE_CHECK_ITERATIONS)
                {
                    if (tfod != null)
                    {
                        // getUpdatedRecognitions() will return null if no new information is available since
                        // the last time that call was made.
                        sleep(1000);

                        List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                        List<Recognition> skystoneRecognitions = new ArrayList();
                        List<Recognition> stoneRecognitions = new ArrayList();


                        if (updatedRecognitions != null)
                        {
                            telemetry.addData("# Object Detected", updatedRecognitions.size());


                            // step through the list of recognitions and display boundary info.
                            int i = 0;
                            for (Recognition recognition : updatedRecognitions)
                            {
                                if (updatedRecognitions.size() < 2)
                                {
                                    if(shuffleCount <= 15)
                                    {
                                        robot.move(0.25, 0.25, false);

                                    }
                                    else
                                        robot.move(0.25, 0.25, true);
                                }

                                telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                                telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f", recognition.getLeft(), recognition.getTop());
                                telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f", recognition.getRight(), recognition.getBottom());

                                if (recognition.getLabel().equals("Skystone"))
                                {
                                    skystoneRecognitions.add(recognition);
                                    telemetry.addData("Skystone Recognitions", skystoneRecognitions.get(0));
                                }
                                else if (recognition.getLabel().equals("Stone"))
                                {
                                    stoneRecognitions.add(recognition);
                                    telemetry.addData("Stone Recognitions", stoneRecognitions.get(0));
                                }
                            }

                            if (updatedRecognitions.size() >= 2)
                            {
                                if (skystoneRecognitions.size() == 0)
                                {
                                    noneTally += 1;
                                    totalTally += 1;
                                    telemetry.addData("None Tally: ", noneTally);
                                }
                                else if (skystoneRecognitions.get(0).getRight() < stoneRecognitions.get(0).getRight())
                                {
                                    leftTally += 1;
                                    totalTally += 1;
                                    telemetry.addData("Left Tally: ", leftTally);
                                }
                                else if (skystoneRecognitions.get(0).getRight() > stoneRecognitions.get(0).getRight())
                                {
                                    rightTally += 1;
                                    totalTally += 1;
                                    telemetry.addData("Right Tally: ", rightTally);
                                }
                                else
                                    telemetry.addData("Error: No Skystone Location Found", "");

                            }

                            telemetry.update();

                            //if skystone is detected and left is less than right, position 1
                            //esle if skystone is detected and right is greater than left, position 2
                            //else (no skystone means position 3

                            //In case we stop the program early, stop tfod
                            if (!opModeIsActive() && tfod != null)
                            {
                                tfod.shutdown();
                            }
                        }
                    }
                }

                if (noneTally == IMAGE_CHECK_ITERATIONS)
                {
                    skystonePosition = Position.none;
                    if (tfod != null) tfod.shutdown();
                    telemetry.addData("Skystone Position: ", "none");
                }
                else if (leftTally == IMAGE_CHECK_ITERATIONS)
                {
                    skystonePosition = Position.left;
                    if (tfod != null) tfod.shutdown();
                    telemetry.addData("Skystone Position: ", "left");
                }
                else if (rightTally == IMAGE_CHECK_ITERATIONS)
                {
                    skystonePosition = Position.right;
                    if (tfod != null) tfod.shutdown();
                    telemetry.addData("Skystone Position: ", "right");
                }
                else
                    telemetry.addData("Error: ", "Tally Check Failed");

            }
        }

    }

    public void shuffle()
    {
        telemetry.addData("Shuffle Count", shuffleCount);
        //Move Shuffle
        if (shuffleCount == 0)
            robot.move(2, 0.1, true);
        else if (shuffleCount == 1)
            robot.move(2, 0.1, true);
        else if (shuffleCount == 2)
            robot.move(2, 0.1, true);
        else if (shuffleCount == 3)
            robot.move(2, 0.1, false);
        else if (shuffleCount == 4)
            robot.move(2, 0.1, false);
        else if (shuffleCount == 5)
            robot.move(2, 0.1, false);

        //Turn Shuffle
        if (shuffleCount == 6)
            robot.turn(10, 0.1, true, false);
        else if (shuffleCount == 8)
            robot.turn(10, 0.1, true, false);
        else if (shuffleCount == 10)
            robot.turn(10, 0.1, true, true);
        else if (shuffleCount == 12)
            robot.turn(10, 0.1, true, true);


        telemetry.addData("Shuffle Count", shuffleCount);
        telemetry.update();
        shuffleCount++;
    }

    public void tensorFlowDrive()
    {
        if (skystonePosition == Position.none)
        {
            telemetry.addData("move to the skystone offscreen", "");
            skystoneNone();
        }
        else if (skystonePosition == Position.left)
        {
            telemetry.addData("move to the left skystone position", "");
            skystoneLeft();
        }
        else if (skystonePosition == Position.right)
        {
            telemetry.addData("move to the right skystone position", "");
            skystoneRight();
        }
        else
            telemetry.addData("Error: ", "No Move");
        telemetry.update();
    }

    //Initialize the Vuforia localization engine.
    public void initVuforia()
    {
        //Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.

        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.

        telemetry.addData("initVuforia: ", "We have vision");
        telemetry.update();
    }

    //Initialize the TensorFlow Object Detection engine.
    public void initTfod()
    {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minimumConfidence = 0.8;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
        telemetry.addData("initTfod: ", "We have vision");
        telemetry.update();
    }
}

