
package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.SerialNumber;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class RobotMecanum// extends Robot
{
    final int TICK_PER_REVOLUTION = 1440;
    final double WHEEL_DIAMETER = 4;

    DcMotor lift;
    final double MAX_LIFT_SPEED = 0.8;

    //Define Wheel Motors
    DcMotor frontLeftWheel;
    DcMotor backLeftWheel;
    DcMotor frontRightWheel;
    DcMotor backRightWheel;

    Servo leftHook;
    Servo rightHook;
    final double LEFT_HOOK_HOME = 0.5;
    final double RIGHT_HOOK_HOME = 0.5;
    final double LEFT_HOOK_EXTENDED = 1.0;
    final double RIGHT_HOOK_EXTENDED = 1.0;
    double leftHookPosition = LEFT_HOOK_HOME;
    double rightHookPosition =  RIGHT_HOOK_HOME;


    Servo claw;
    final double CLAW_HOME = 0.0;
    final double CLAW_EXTENDED = 0.38;
    double clawPosition = CLAW_HOME;


    //Sensors
    GyroSensor gyro;
    ModernRoboticsI2cRangeSensor rangeSensorRightFront;
    ModernRoboticsI2cRangeSensor rangeSensorRightBack;
    ModernRoboticsI2cRangeSensor rangeSensorLeftFront;
    ModernRoboticsI2cRangeSensor rangeSensorLeftBack;

    //ColorSensor colorSensorRight;
    //ColorSensor colorSensorLeft;

    //=======================================================
    OpMode opMode;
    HardwareMap hardwareMap;
    //LinearOpMode opMode;
    Telemetry telemetry;

    TensorFlow tensorFlow;

    //==============================================================================================   Robot method
    public RobotMecanum(HardwareMap hMap, OpMode opMode)
    {

        hardwareMap = hMap;
        this.opMode = opMode;
        //this.opMode = (LinearOpMode) opMode;
        telemetry = opMode.telemetry;

        //super(hMap, opMode);

        lift = hardwareMap.dcMotor.get("lift");

        frontLeftWheel = hardwareMap.dcMotor.get("front_left_wheel");
        backLeftWheel = hardwareMap.dcMotor.get("back_left_wheel");
        frontRightWheel = hardwareMap.dcMotor.get("front_right_wheel");
        backRightWheel = hardwareMap.dcMotor.get("back_right_wheel");

        leftHook = hardwareMap.servo.get("left_hook");
        rightHook = hardwareMap.servo.get("right_hook");

        leftHook.setDirection(Servo.Direction.REVERSE);

        //Reverse the two flipped wheels
        frontRightWheel.setDirection(DcMotor.Direction.REVERSE);
        backRightWheel.setDirection(DcMotor.Direction.REVERSE);

        //Claw
        claw = hardwareMap.servo.get("claw");
        claw.setDirection(Servo.Direction.REVERSE);
        //claw.setPosition(CLAW_HOME);

        rangeSensorRightFront = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "range_sensor_right_front");
        rangeSensorRightBack = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "range_sensor_right_back");
        rangeSensorLeftFront = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "range_sensor_left_front");
        rangeSensorLeftBack = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "range_sensor_left_back");

        rangeSensorRightFront.setI2cAddress(I2cAddr.create8bit(0X78));
        rangeSensorRightBack.setI2cAddress(I2cAddr.create8bit(0X76));
        rangeSensorLeftFront.setI2cAddress(I2cAddr.create8bit(0X28));
        rangeSensorLeftBack.setI2cAddress(I2cAddr.create8bit(0X26));


        gyro = hardwareMap.gyroSensor.get("gyro");
        gyro.calibrate();
    }
    //==============================================================================================   convertDistTicks
    //method takes in 2nd parameter for circumfrence of spinning object
    public int convertDistTicks(double distanceToTravel, double circumfrence)
    {
        //1440 revolutions = 1 rotation
        //1 rotation = 4

        double revolutions = distanceToTravel / circumfrence;
        int totalTicks = (int) Math.round(revolutions * TICK_PER_REVOLUTION);

        return totalTicks;
    }
    public void moveOmni(double drivePower, double strafePower, double rotatePower)
    {
        double drive = Math.signum(-drivePower) * Math.pow(drivePower, 4);
        double strafe = Math.signum(strafePower) * Math.pow(strafePower, 4);
        double rotate = rotatePower;

        double frontLeftPower = drive + strafe + rotate;
        double backLeftPower = drive - strafe + rotate;
        double frontRightPower = drive - strafe - rotate;
        double backRightPower = drive + strafe - rotate;

        //frontLeftPower = drive + strafe + rotate;
        //backLeftPower = drive - strafe + rotate;
        //frontRightPower = drive - strafe - rotate;
        //backRightPower = drive + strafe - rotate;

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
    public void drive (double distance, double power)
    {
        frontLeftWheel.setTargetPosition(convertDistTicks(distance, WHEEL_DIAMETER * Math.PI));
        backLeftWheel.setTargetPosition(convertDistTicks(distance, WHEEL_DIAMETER * Math.PI));
        frontRightWheel.setTargetPosition(convertDistTicks(distance, WHEEL_DIAMETER * Math.PI));
        backRightWheel.setTargetPosition(convertDistTicks(distance, WHEEL_DIAMETER * Math.PI));
        moveOmni( power, 0, 0);
    }
    public void strafe(double distance, double power)
    {
        frontLeftWheel.setTargetPosition(convertDistTicks(distance, WHEEL_DIAMETER * Math.PI));
        backLeftWheel.setTargetPosition(convertDistTicks(distance, WHEEL_DIAMETER * Math.PI));
        frontRightWheel.setTargetPosition(convertDistTicks(distance, WHEEL_DIAMETER * Math.PI));
        backRightWheel.setTargetPosition(convertDistTicks(distance, WHEEL_DIAMETER * Math.PI));
        moveOmni( 0, power, 0);
    }
    public void rotate(double distance, double power)
    {
        frontLeftWheel.setTargetPosition(convertDistTicks(distance, WHEEL_DIAMETER * Math.PI));
        backLeftWheel.setTargetPosition(convertDistTicks(distance, WHEEL_DIAMETER * Math.PI));
        frontRightWheel.setTargetPosition(convertDistTicks(distance, WHEEL_DIAMETER * Math.PI));
        backRightWheel.setTargetPosition(convertDistTicks(distance, WHEEL_DIAMETER * Math.PI));

        moveOmni(0, 0, power);
    }

    /**
     *
     * @param distanceFromWall distance to stop from the wall in inches
     * @param power power to run the motors. + is to the right, - is to the left
     */
    public void strafeRange(int distanceFromWall, double power){
        frontRightWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLeftWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        if(power > 0){
            moveOmni(0,power,0);
            while(rangeSensorRightFront.getDistance(DistanceUnit.INCH) > distanceFromWall || rangeSensorRightBack.getDistance(DistanceUnit.INCH) > distanceFromWall){
                telemetry.addData("distance from wall", rangeSensorRightFront.getDistance(DistanceUnit.INCH));
                telemetry.update();
            }
            moveOmni(0,0,0);
        }

        else{
            moveOmni(0,power,0);
            while(rangeSensorLeftFront.getDistance(DistanceUnit.INCH) > distanceFromWall || rangeSensorLeftBack.getDistance(DistanceUnit.INCH) > distanceFromWall){
                telemetry.addData("distance from wall", rangeSensorLeftFront.getDistance(DistanceUnit.INCH));
                telemetry.update();
            }
            moveOmni(0,0,0);
        }

    }

    public void incrementalDrive(double distance, double power, boolean isStrafe)
    {
        for(int i = 0; i < distance/2; i++)
        {
            if (isStrafe)
                strafe(2, power);
            else
                drive(2, power);
        }
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

        telemetry.addData("Claw Position: ", claw.getPosition());
        telemetry.update();
    }
    public void hooks(boolean hooksHome)
    {
        telemetry.addData("hooks", "running");
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

        telemetry.addData("Left Hook Position: ", leftHook.getPosition());
        telemetry.addData("Right Hook Position: ", rightHook.getPosition());
        telemetry.update();
    }
    //==============================================================================================   turnGyro
    //not ready or programmed yet
    public void turnGyro(double turningDegrees, double power, boolean turnRight)
    {
        telemetry.addData("turnGyro", "running");
        telemetry.update();

        gyro.resetZAxisIntegrator();
        if(turnRight)
        {
            //mostly works --- turned right 270 when I wanted it to move 90 right at 0.7 power
            while(gyro.getHeading() + 180 < 180 + turningDegrees)
            {
                moveOmni(0, 0, power);
            }
        }
        //doesn't work --- had to leave didn't get to finishing this up.
        else
        {
            while(gyro.getHeading() + 180 < 180 - turningDegrees)
            {
                moveOmni(0, 0, -power);
            }
        }

        moveOmni(0, 0, 0);

    }
}

