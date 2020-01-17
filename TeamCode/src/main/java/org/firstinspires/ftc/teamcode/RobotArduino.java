package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class RobotArduino
{
    //======================================================
    DcMotor leftWheel;
    DcMotor rightWheel;

    //======================================================
    final int tickPerRevlolution = 1440;
    final double linearWheelDistance = (Math.PI) * 1;//.314961;

    ModernRoboticsI2cColorSensor colorSensorRight;
    ModernRoboticsI2cColorSensor colorSensorMiddleRight;
    ModernRoboticsI2cColorSensor colorSensorMiddleLeft;
    ModernRoboticsI2cColorSensor colorSensorLeft;

    //======================================================
    enum Position{none,left,right};

    HardwareMap hardwareMap;
    OpMode opMode;
    //LinearOpMode opMode;
    Telemetry telemetry;

    //==============================================================================================   RobotArduino method
    public RobotArduino(HardwareMap hMap, OpMode opMode)
    {
        hardwareMap = hMap;
        this.opMode = opMode;
        //this.opMode = (LinearOpMode) opMode;
        telemetry = opMode.telemetry;

        leftWheel = hardwareMap.dcMotor.get("back_left_motor");
        rightWheel = hardwareMap.dcMotor.get("back_right_motor");

        colorSensorRight = hardwareMap.get(ModernRoboticsI2cColorSensor.class, "color_sensor_right");
        colorSensorMiddleRight = hardwareMap.get(ModernRoboticsI2cColorSensor.class, "color_sensorMiddle_Right");
        colorSensorMiddleLeft = hardwareMap.get(ModernRoboticsI2cColorSensor.class, "color_sensorMiddle_Left");
        colorSensorLeft = hardwareMap.get(ModernRoboticsI2cColorSensor.class, "color_sensor_Left");

        /*colorSensorRight = hardwareMap.get(ModernRoboticsI2cColorSensor.class, "color_sensor_right");
        colorSensorMiddleRight = hardwareMap.get(ModernRoboticsI2cColorSensor.class, "color_sensorMiddle_Right");
        colorSensorMiddleLeft = hardwareMap.get(ModernRoboticsI2cColorSensor.class, "color_sensorMiddle_Left");
        colorSensorLeft = hardwareMap.get(ModernRoboticsI2cColorSensor.class, "color_sensor_Left");*/

        telemetry.addData("RobotArduino", "finished setting up hardware");
        telemetry.update();
    }
    //==============================================================================================   convertDistTicks
    //method takes in 2nd parameter for circumfrence of spinning object
    public int convertDistTicks(double distanceToTravel, double circumfrence)
    {
        //1440 revolutions = 1 rotation
        //1 rotation = 4

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
        leftWheel.setDirection(DcMotor.Direction.REVERSE);
        rightWheel.setDirection(DcMotor.Direction.FORWARD);

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
        while ( leftWheel.isBusy() )
        {
            telemetry.addData("encoder-fwd", leftWheel.getCurrentPosition() + "  busy=" + leftWheel.isBusy() );
            telemetry.update();
        }
        while ( rightWheel.isBusy() )
        {
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


}
