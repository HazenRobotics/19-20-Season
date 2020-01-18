package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class RobotArduino
{
    long setTime = System.currentTimeMillis();
    boolean  hasRun = false;

    double previousTime;

    //======================================================
    float[] hsvValuesA = {0F,0F,0F};
    float[] hsvValuesC = {0F,0F,0F};
    float[] hsvValuesE = {0F,0F,0F};

    final boolean bLedOn = true;
    final boolean bLedOff = false;

    //======================================================
    DcMotor leftWheel;
    DcMotor rightWheel;

    //======================================================
    final int tickPerRevlolution = 1440;
    // this is the curcumference which is pi time diameter
    final double linearWheelDistance = (Math.PI) * 8;//.314961;

    ModernRoboticsI2cColorSensor colorSensorRight;
    ModernRoboticsI2cColorSensor colorSensorMiddle;
    ModernRoboticsI2cColorSensor colorSensorLeft;

    //======================================================
    enum Position{none,left,right};

    OpMode opMode;
    HardwareMap hardwareMap;
    //LinearOpMode opMode;
    Telemetry telemetry;

    //==============================================================================================   RobotArduino method
    public RobotArduino(HardwareMap hMap, OpMode opMode)
    {
        this.hardwareMap = hMap;
        this.opMode = opMode;
        //this.opMode = (LinearOpMode) opMode;
        this.telemetry = opMode.telemetry;

        leftWheel = hardwareMap.dcMotor.get("left_motor");
        rightWheel = hardwareMap.dcMotor.get("right_motor");

        leftWheel.setDirection(DcMotor.Direction.FORWARD);
        rightWheel.setDirection(DcMotor.Direction.REVERSE);

        leftWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        colorSensorLeft = hardwareMap.get(ModernRoboticsI2cColorSensor.class, "color_sensor_left");
        colorSensorMiddle = hardwareMap.get(ModernRoboticsI2cColorSensor.class, "color_sensor_middle");
        colorSensorRight = hardwareMap.get(ModernRoboticsI2cColorSensor.class, "color_sensor_right");

        colorSensorLeft.setI2cAddress(I2cAddr.create8bit(0X3A));
        colorSensorMiddle.setI2cAddress(I2cAddr.create8bit(0X3C));
        colorSensorRight.setI2cAddress(I2cAddr.create8bit(0X3E));

        telemetry.addData("RobotArduino", "finished setting up hardware");
        telemetry.update();
    }
    //==============================================================================================   convertDistTicks
    //method takes in 2nd parameter for circumfrence of spinning object
    public int convertDistTicks(double distanceToTravel, double circumfrence)
    {
        // 1440 revolutions = 1 rotation
        // 1 rotation = 4

        double revolutions = distanceToTravel / circumfrence;
        int totalTicks = (int) Math.round(revolutions * tickPerRevlolution);

        return totalTicks;
    }
    //==============================================================================================   move
    public void move(double distanceToTravel,double power, boolean isForward)
    {
        telemetry.addData("move method", "running");
        telemetry.update();

        // reset encoder count kept by left motor.
        leftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftWheel.setDirection(DcMotor.Direction.FORWARD);
        rightWheel.setDirection(DcMotor.Direction.FORWARD);

        if (isForward)
            rightWheel.setDirection(DcMotor.Direction.REVERSE);
        else
            leftWheel.setDirection(DcMotor.Direction.REVERSE);

        // set left motor to run to 5000 encoder counts.
        leftWheel.setTargetPosition(convertDistTicks(distanceToTravel, linearWheelDistance));
        rightWheel.setTargetPosition(convertDistTicks(distanceToTravel, linearWheelDistance));

        // set both motors to power. Movement will start.
        leftWheel.setPower(power);
        rightWheel.setPower(power);

        // set left motor to run to target encoder position and stop with brakes on.
        leftWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // wait while opmode is active and motors are busy running to position.

        while (leftWheel.isBusy())
        {
            telemetry.addData("encoder-fwd", leftWheel.getCurrentPosition()
                    + "  busy=" + leftWheel.isBusy());
            telemetry.update();
        }
        while (rightWheel.isBusy())
        {
            telemetry.addData("encoder-fwd", rightWheel.getCurrentPosition()
                    + "  busy=" + rightWheel.isBusy());
            telemetry.update();
        }

        // set motor power to zero to turn off motors. The motors stop on their own but
        // power is still applied so we turn off the power.
        leftWheel.setPower(0.0);
        rightWheel.setPower(0.0);
    }
    //==============================================================================================   turnOnSpot
    public void turnOnSpot(double turningDegrees, double power, boolean turnLeft)
    {
        telemetry.addData("turnOnSpot", "running");
        telemetry.update();

        double turningNumber = (turningDegrees/180) * 16.4 * (Math.PI);
        double onSpotTurningNumber = turningNumber/2;

        // reset encoder count kept by left motor.
        leftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftWheel.setDirection(DcMotor.Direction.REVERSE);
        rightWheel.setDirection(DcMotor.Direction.REVERSE);

        //forward direction switching both wheels for turning in spot
        if(turnLeft)
        {
            leftWheel.setDirection(DcMotor.Direction.FORWARD);
            rightWheel.setDirection(DcMotor.Direction.FORWARD);
        }

        // reset encoder count kept by left motor.
        leftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // set left motor to run to 5000 encoder counts.
        leftWheel.setTargetPosition(convertDistTicks(onSpotTurningNumber,linearWheelDistance));
        rightWheel.setTargetPosition(convertDistTicks(onSpotTurningNumber,linearWheelDistance));

        // set both motors to 25% power. Movement will start.
        leftWheel.setPower(power);
        rightWheel.setPower(power);

        // set left motor to run to target encoder position and stop with brakes on.
        leftWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // wait while opmode is active and left motor is busy running to position.
        while ( leftWheel.isBusy() && rightWheel.isBusy() )
        {
            telemetry.addData("encoder-fwd", leftWheel.getCurrentPosition() + "  busy=" + leftWheel.isBusy() );
            telemetry.addData("encoder-fwd", rightWheel.getCurrentPosition() + "  busy=" + rightWheel.isBusy() );
            telemetry.update();
        }

        // set motor power to zero to turn off motors. The motors stop on their own but
        // power is still applied so we turn off the power.
        leftWheel.setPower(0.0);
        rightWheel.setPower(0.0);
    }
    //==============================================================================================   turn
    public void turn(double turningDegrees, double power, boolean isForward, boolean usesLeftWheel)
    {
        telemetry.addData("turn", "running");
        telemetry.update();

        // calculations from degrees to motor distance

        // 90* arc length = (radius/2) * pi
        // angle/180 * radius * pi
        // (angle/180) * 16.5 * (Math.PI)
        // (turningDegrees/180) * 16.5 * (Math.PI)

        double turningNumber = (turningDegrees/180) * 16.4 * (Math.PI);

        leftWheel.setDirection(DcMotor.Direction.FORWARD);
        rightWheel.setDirection(DcMotor.Direction.FORWARD);

        if (isForward)
            rightWheel.setDirection(DcMotor.Direction.REVERSE);
        else
            leftWheel.setDirection(DcMotor.Direction.REVERSE);

        //if wheel is left:
        if(usesLeftWheel)
        {
            // reset encoder count kept by left motor.
            leftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            // set left motor to run to 5000 encoder counts.
            leftWheel.setTargetPosition(convertDistTicks(turningNumber,linearWheelDistance));

            // set both motors to 25% power. Movement will start.
            leftWheel.setPower(power);

            // set left motor to run to target encoder position and stop with brakes on.
            leftWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // wait while opmode is active and left motor is busy running to position.
            while ( leftWheel.isBusy() )
            {
                telemetry.addData("encoder-fwd", rightWheel.getCurrentPosition() + "  busy=" + rightWheel.isBusy() );
                telemetry.update();
            }

            // set motor power to zero to turn off motors. The motors stop on their own but
            // power is still applied so we turn off the power.
            leftWheel.setPower(0.0);
        }

        //if wheel is right:
        else
        {
            // reset encoder count kept by left motor.
            rightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            // set left motor to run to 5000 encoder counts.
            rightWheel.setTargetPosition(convertDistTicks(turningNumber,linearWheelDistance));

            // set both motors to 25% power. Movement will start.
            rightWheel.setPower(power);

            // set left motor to run to target encoder position and stop with brakes on.
            rightWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // wait while opmode is active and left motor is busy running to position.
            while ( rightWheel.isBusy() )
            {
                telemetry.addData("encoder-fwd", rightWheel.getCurrentPosition() + "  busy=" + rightWheel.isBusy() );
                telemetry.update();
            }

            // set motor power to zero to turn off motors. The motors stop on their own but
            // power is still applied so we turn off the power.
            rightWheel.setPower(0.0);
        }
    }
    public void sleepRobot(long delay)
    {
        long setTime = System.currentTimeMillis();
        previousTime = opMode.getRuntime();

        while(System.currentTimeMillis() - setTime < (delay) && ((LinearOpMode) opMode).opModeIsActive() )
            previousTime = opMode.getRuntime();

        telemetry.addData("Finished Sleep", "");
        telemetry.update();
    }

    public boolean colorSensorIsBlack(ModernRoboticsI2cColorSensor colorSensor)
    {
        boolean colorIsBlack = false;

        colorSensor.enableLed(bLedOn);

        colorSensor = somewhatInitiateSensorColors(colorSensor);

        if ( colorSensor.red() < 5 && colorSensor.green() < 5 && colorSensor.blue() < 5 )
            colorIsBlack = true;

        return colorIsBlack;
    }

    public ModernRoboticsI2cColorSensor somewhatInitiateSensorColors(ModernRoboticsI2cColorSensor colorSensor)
    {
        Color.RGBToHSV(colorSensorLeft.red() * 8, colorSensorLeft.green() * 8, colorSensorLeft.blue() * 8, hsvValuesA);
        Color.RGBToHSV(colorSensorMiddle.red() * 8, colorSensorMiddle.green() * 8, colorSensorMiddle.blue() * 8, hsvValuesC);
        Color.RGBToHSV(colorSensorRight.red() * 8, colorSensorRight.green() * 8, colorSensorRight.blue() * 8, hsvValuesE);

        return colorSensor;
    }

    public boolean needToTurnSlightly()
    {
        boolean needToTurnSlightly = false;

        if (  colorSensorIsBlack( colorSensorMiddle )  &&   (  colorSensorIsBlack( colorSensorMiddle )  ||  colorSensorIsBlack( colorSensorMiddle )  )   )
            needToTurnSlightly = true;

        return needToTurnSlightly;
    }

    public boolean needToTurnModerately()
    {
        boolean needToTurnModerately = false;

        if (  colorSensorIsBlack( colorSensorMiddle )  &&   (  colorSensorIsBlack( colorSensorMiddle )  ||  colorSensorIsBlack( colorSensorMiddle )  )   )
            needToTurnModerately = true;

        return needToTurnModerately;
    }
    public boolean needToTurnLeft()
    {
        boolean needToTurnLeft = false;

        if (  colorSensorIsBlack( colorSensorLeft )  )
            needToTurnLeft = true;

        return needToTurnLeft;
    }

    public void turningMovements()
    {
        double turningPower = 0.5;
        //int turningDegrees = 5;
        boolean usesLeftWheel = false;

        if ( needToTurnLeft() )
            usesLeftWheel = true;

        /*if ( needToTurnModerately() )
            turningDegrees *= 2;*/

        //turn(turningDegrees, turningPower, true, usesLeftWheel);
        //turn(0, 0, true, true);

        turnTime(0.6, 200, usesLeftWheel);

        telemetry.addData("turningMovements", "finished");
        telemetry.update();
    }
    public void moveTime(double power, long time)
    {
        //wait for certain amount of time while motors are running
        //robotMecanum.wait(time);
        long setTime = System.currentTimeMillis();
        previousTime = opMode.getRuntime();

        while( System.currentTimeMillis() - setTime < (time) && ((LinearOpMode) opMode).opModeIsActive() )
        {
            leftWheel.setPower(power);
            rightWheel.setPower(power);
        }

        //sets all power to zero afterwords
        leftWheel.setPower(0);
        rightWheel.setPower(0);
    }
    public void turnTime(double power, long time, boolean usesLeftWheel)
    {
        //wait for certain amount of time while motors are running
        //robotMecanum.wait(time);
        long setTime = System.currentTimeMillis();
        previousTime = opMode.getRuntime();

        if(usesLeftWheel)
            while( System.currentTimeMillis() - setTime < (time) && ((LinearOpMode) opMode).opModeIsActive() )
                leftWheel.setPower(power);
        else
            while( System.currentTimeMillis() - setTime < (time) && ((LinearOpMode) opMode).opModeIsActive() )
                rightWheel.setPower(power);

        //sets all power to zero afterwords
        leftWheel.setPower(0);
        rightWheel.setPower(0);
    }

}
