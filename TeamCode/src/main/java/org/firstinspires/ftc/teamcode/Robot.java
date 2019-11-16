
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

public class Robot
{
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

    //======================================================
    int shuffleCount = 0;

    HardwareMap hardwareMap;
    OpMode opMode;
    Telemetry telemetry;

    TensorFlow tensorFlow;

    //==============================================================================================

    //=========================================================================================
    //Robot method
    public Robot(HardwareMap hardwareMap, OpMode opMode)
    {
        //telemetry.addData("Robot", "setting up hardware");
        //telemetry.update();

        //arm = hardwareMap.servo.get("arm");
        this.hardwareMap = hardwareMap;
        this.opMode = opMode;
        telemetry = opMode.telemetry;

        //Map Hardware
        lift = hardwareMap.dcMotor.get("lift");

        rightMotor = hardwareMap.dcMotor.get("right_wheel");

        leftHook = hardwareMap.servo.get("left_hook");
        rightHook = hardwareMap.servo.get("right_hook");

        leftClapper = hardwareMap.servo.get("left_clapper");
        rightClapper = hardwareMap.servo.get("right_clapper");

        gyro = hardwareMap.gyroSensor.get("gyro");

        gyro.calibrate();

        tensorFlow = new TensorFlow(hardwareMap, opMode);
    }
    //=========================================================================================
    //Lift method
    public void setlift(double liftPower)
    {
        //telemetry.addData("setLift", "running");
        //telemetry.update();

        convertDistTicks(5.5, linearSpoolDistance);
    }


    //==========================================================================================
    //clapper method
    public void clapper(boolean clappersHome)
    {
        //telemetry.addData("clappers", "running");
        //telemetry.update();

        //set clappers position to their positions
        if(clappersHome)
        {
            leftClapperPosition = LEFT_CLAPPER_HOME;
            rightClapperPosition = RIGHT_CLAPPER_HOME;
        }
        else
        {
            leftClapperPosition = LEFT_CLAPPER_EXTENDED;
            rightClapperPosition = RIGHT_CLAPPER_EXTENDED;
        }
        leftClapper.setPosition(leftClapperPosition);
        rightClapper.setPosition(rightClapperPosition);
    }


    //==========================================================================================
    //hook methd
    public void hooks(boolean hooksHome)
    {
        //telemetry.addData("hooks", "running");
        //telemetry.update();

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

    //method takes in 2nd parameter for circumfrence of spinning object
    public int convertDistTicks(double distanceToTravel, double circumfrence)
    {
        //1440 revolutions = 1 rotation
        //1 rotation = 4

        double revolutions = distanceToTravel / circumfrence;
        int totalTicks = (int) Math.round(revolutions * tickPerRevlolution);

        return totalTicks;
    }

    public void move(double distanceToTravel,double power, boolean isForward)
    {
        //telemetry.addData("move", "running");
        //telemetry.update();

        // reset encoder count kept by left motor.
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        if (isForward)
        {
            rightMotor.setDirection(DcMotor.Direction.REVERSE);
            leftMotor.setDirection(DcMotor.Direction.FORWARD);
        }
        else
        {
            rightMotor.setDirection(DcMotor.Direction.FORWARD);
            leftMotor.setDirection(DcMotor.Direction.REVERSE);
        }

        // set left motor to run to 5000 encoder counts.
        leftMotor.setTargetPosition(convertDistTicks(distanceToTravel, linearWheelDistance));
        rightMotor.setTargetPosition(convertDistTicks(distanceToTravel, linearWheelDistance));

        // set both motors to 25% power. Movement will start.
        leftMotor.setPower(power);
        rightMotor.setPower(power);

        // set left motor to run to target encoder position and stop with brakes on.
        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // wait while opmode is active and motors are busy running to position.

        while (leftMotor.isBusy())
        {
            telemetry.addData("encoder-fwd", leftMotor.getCurrentPosition() + "  busy=" + leftMotor.isBusy());
            telemetry.update();
        }
        while (rightMotor.isBusy())
        {
            telemetry.addData("encoder-fwd", rightMotor.getCurrentPosition() + "  busy=" + rightMotor.isBusy());
            telemetry.update();
        }

        // set motor power to zero to turn off motors. The motors stop on their own but
        // power is still applied so we turn off the power.
        leftMotor.setPower(0.0);
        rightMotor.setPower(0.0);
    }



    public void turnOnSpot(double turningDegrees, double power, boolean turnLeft)
    {
        //telemetry.addData("turnOnSpot", "running");
        //telemetry.update();

        double turningNumber = (turningDegrees/180) * 16.4 * (Math.PI);
        double onSpotTurningNumber = turningNumber/2;

        // reset encoder count kept by left motor.
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //forward direction switching both wheels for turning in spot
        if(turnLeft)
        {
            rightMotor.setDirection(DcMotor.Direction.FORWARD);
            leftMotor.setDirection(DcMotor.Direction.FORWARD);
        }
        else
        {
            rightMotor.setDirection(DcMotor.Direction.REVERSE);
            leftMotor.setDirection(DcMotor.Direction.REVERSE);
        }
    }


    //turning method
    public void turn(double turningDegrees, double power, boolean isForward, boolean leftWheel)
    {
        //telemetry.addData("turn", "running");
        //telemetry.update();

        // calculations from degrees to motor distance

        // 90* arc length = (radius/2) * pi
        // angle/180 * radius * pi
        // (angle/180) * 16.5 * (Math.PI)
        // (turningDegrees/180) * 16.5 * (Math.PI)

        double turningNumber = (turningDegrees/180) * 16.4 * (Math.PI);
        double onSpotTurningNumber = turningNumber/2;

        if (isForward)
        {
            rightMotor.setDirection(DcMotor.Direction.REVERSE);
            leftMotor.setDirection(DcMotor.Direction.FORWARD);

        }
        else
        {
            rightMotor.setDirection(DcMotor.Direction.FORWARD);
            leftMotor.setDirection(DcMotor.Direction.REVERSE);
        }

        //if wheel is left:
        if(leftWheel)
        {
            // reset encoder count kept by left motor.
            leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            // set left motor to run to 5000 encoder counts.
            leftMotor.setTargetPosition(convertDistTicks(turningNumber,linearWheelDistance));

            // set both motors to 25% power. Movement will start.
            leftMotor.setPower(power);

            // set left motor to run to target encoder position and stop with brakes on.
            leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // wait while opmode is active and left motor is busy running to position.
            while (leftMotor.isBusy()) {
                telemetry.addData("encoder-fwd", leftMotor.getCurrentPosition() + "  busy=" + leftMotor.isBusy());
                telemetry.update();
            }

            // set motor power to zero to turn off motors. The motors stop on their own but
            // power is still applied so we turn off the power.
            leftMotor.setPower(0.0);
        }

        //if wheel is right:
        else
        {
            // reset encoder count kept by left motor.
            rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            // set left motor to run to 5000 encoder counts.
            rightMotor.setTargetPosition(convertDistTicks(turningNumber,linearWheelDistance));

            // set both motors to 25% power. Movement will start.
            rightMotor.setPower(power);

            // set left motor to run to target encoder position and stop with brakes on.
            rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // wait while opmode is active and left motor is busy running to position.
            while (rightMotor.isBusy())
            {
                //telemetry.addData("encoder-fwd", rightMotor.getCurrentPosition() + "  busy=" + rightMotor.isBusy());
                //telemetry.update();
            }

            // set motor power to zero to turn off motors. The motors stop on their own but
            // power is still applied so we turn off the power.
            leftMotor.setPower(0.0);
            rightMotor.setPower(0.0);
        }
    }

    public void turnGyro(double turningDegrees, double power, boolean turnRight)
    {
        telemetry.addData("turnGyro", "running");
        telemetry.update();

        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        gyro.resetZAxisIntegrator();
        if(turnRight)
        {
            rightMotor.setDirection(DcMotor.Direction.FORWARD);
            leftMotor.setDirection(DcMotor.Direction.FORWARD);

            rightMotor.setPower(power);
            leftMotor.setPower(power);

            while(gyro.getHeading() + 180 < 180 - turningDegrees) {}
        }
        else
        {
            rightMotor.setDirection(DcMotor.Direction.REVERSE);
            leftMotor.setDirection(DcMotor.Direction.REVERSE);

            rightMotor.setPower(power);
            leftMotor.setPower(power);

            while(gyro.getHeading() + 180 < 180 + turningDegrees) {}
        }

        rightMotor.setPower(0);
        leftMotor.setPower(0);

        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    //==========================================================================================
    //skystoneNone method
    public void skystoneNone()
    {
        telemetry.addData("skystoneNone", "running");
        telemetry.update();
        //Position to pick up skystone

        //Dive and pick up skystone
        move(12, 1, true);


        clapper(false);

        //Back up with skystone and rotate
        move(12, 1, false);

        //move to the preplanned position
        move(12, 1, true);

        //Run method to return place skystone on foundation
        skystoneReturn();
    }

    public void skystoneLeft()
    {
        telemetry.addData("skystoneLeft", "running");
        telemetry.update();
        //Position to pick up skystone

        //Dive and pick up skystone
        move(12, 1, true);

        clapper(false);

        //Back up with skystone and rotate
        move(12, 1, false);

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
        move(8, 1, true);
        clapper(false);

        //Back up with skystone and rotate
        move(6, 1, false);

        //move to the preplanned position
        turn(90,1,false,false);

        move(6, 1, true);

        //Run method to return place skystone on foundation
        skystoneReturn();
    }

    public void skystoneReturn()
    {
        move(24, 1, true);



        move(12, 1, true);
    }
    public void shuffle()
    {
        telemetry.addData("Shuffle Count", shuffleCount);
        //Move Shuffle
        if (shuffleCount == 0)
            move(2, 0.1, true);
        else if (shuffleCount == 1)
            move(2, 0.1, true);
        else if (shuffleCount == 2)
            move(2, 0.1, true);
        else if (shuffleCount == 3)
            move(2, 0.1, false);
        else if (shuffleCount == 4)
            move(2, 0.1, false);
        else if (shuffleCount == 5)
            move(2, 0.1, false);

        //Turn Shuffle
        if (shuffleCount == 6)
            turn(10, 0.1, true, false);
        else if (shuffleCount == 8)
            turn(10, 0.1, true, false);
        else if (shuffleCount == 10)
            turn(10, 0.1, true, true);
        else if (shuffleCount == 12)
            turn(10, 0.1, true, true);


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
}

