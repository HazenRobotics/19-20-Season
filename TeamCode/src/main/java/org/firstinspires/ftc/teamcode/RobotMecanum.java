
package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class RobotMecanum// extends LinearOpMode  //Robot
{
    long setTime = System.currentTimeMillis();
    boolean  hasRun = false;

    protected String skystoneLocation = "";

    double P = 0.0135, I = 0.02025, D = 0;
    int integral,previous_error;
    double previousTime;

    final int TICK_PER_REVOLUTION = 1440;
    final double WHEEL_DIAMETER = 4;
    final int GEAR_RATIO = 2;

    final double BACK_SENSOR_SEPERATION = 25/32;

    DcMotor lift;
    final double MAX_LIFT_SPEED = 0.8;
    final int TICKS_PER_BLOCK = 1;

    //Define Wheel Motors
    DcMotor frontLeftWheel;
    DcMotor backLeftWheel;
    DcMotor frontRightWheel;
    DcMotor backRightWheel;

    Servo leftHook;
    Servo rightHook;
    final double LEFT_HOOK_HOME = convertDegreeToPercent(0.0,360.0);
    final double RIGHT_HOOK_HOME = convertDegreeToPercent(0.0,360.0);
    final double LEFT_HOOK_EXTENDED = convertDegreeToPercent(280.0,360.0);
    final double RIGHT_HOOK_EXTENDED = convertDegreeToPercent(280.0,360.0);
    double leftHookPosition = LEFT_HOOK_HOME;
    double rightHookPosition =  RIGHT_HOOK_HOME;

    Servo claw;
    final double CLAW_HOME = convertDegreeToPercent(180.0,360.0);
    final double CLAW_EXTENDED = convertDegreeToPercent(360,360.0);
    double clawPosition = CLAW_HOME;

    Servo capper;
    final double CAPPER_HOME = convertDegreeToPercent(120.0,360.0);
    final double CAPPER_EXTENDED = convertDegreeToPercent(360,360.0);
    double capperPosition = CLAW_HOME;

    GyroSensor gyro;


    //Sensors - Left
    ModernRoboticsI2cRangeSensor rangeSensorRightFront;
    ModernRoboticsI2cRangeSensor rangeSensorRightBack;
    ModernRoboticsI2cRangeSensor rangeSensorLeftFront;
    ModernRoboticsI2cRangeSensor rangeSensorLeftBack;

    //Sensors - Left
    ModernRoboticsI2cRangeSensor rangeSensorBackRight;
    ModernRoboticsI2cRangeSensor rangeSensorBackLeft;

    ColorSensor colorSensorLeft;    // 0x3A
    ColorSensor colorSensorRight;    // 0x3C


    float hsvValues[] = {0F,0F,0F};     // hsvValues is an array that will hold the hue, saturation, and value information.
    final float values[] = hsvValues;   // values is a reference to the hsvValues array.

    boolean bLedOn = true;          // bLedOn represents the state of the LED.

    //=======================================================
    OpMode opMode;
    HardwareMap hardwareMap;
    //LinearOpMode opMode;
    Telemetry telemetry;

    TensorFlow tensorFlow;

    //==============================================================================================   Robot method
    public RobotMecanum(HardwareMap hMap, OpMode opMode, boolean isTeleOP)
    {
        hardwareMap = hMap;
        this.opMode = opMode;
        //this.opMode = (LinearOpMode) opMode;
        telemetry = opMode.telemetry;

        //super(hMap, opMode);

        telemetry.setAutoClear(true);

        lift = hardwareMap.dcMotor.get("lift");

        frontLeftWheel = hardwareMap.dcMotor.get("front_left_wheel");
        backLeftWheel = hardwareMap.dcMotor.get("back_left_wheel");
        frontRightWheel = hardwareMap.dcMotor.get("front_right_wheel");
        backRightWheel = hardwareMap.dcMotor.get("back_right_wheel");

        leftHook = hardwareMap.servo.get("left_hook");
        rightHook = hardwareMap.servo.get("right_hook");

        leftHook.setDirection(Servo.Direction.REVERSE);

        //Reverse the two flipped wheels
        frontLeftWheel.setDirection(DcMotor.Direction.REVERSE);
        backLeftWheel.setDirection(DcMotor.Direction.REVERSE);

        //Claw
        claw = hardwareMap.servo.get("claw");
        claw.setDirection(Servo.Direction.REVERSE);

        //capper
        capper = hardwareMap.servo.get("capper");

        if(!isTeleOP)
        {
            gyro = hardwareMap.gyroSensor.get("gyro");
            gyro.calibrate();
            while (gyro.isCalibrating()) ;

            tensorFlow = new TensorFlow(hardwareMap, opMode);

            rangeSensorRightFront = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "range_sensor_right_front");
            rangeSensorRightBack = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "range_sensor_right_back");
            rangeSensorLeftFront = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "range_sensor_left_front");
            rangeSensorLeftBack = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "range_sensor_left_back");

            rangeSensorRightFront.setI2cAddress(I2cAddr.create8bit(0X78));
            rangeSensorRightBack.setI2cAddress(I2cAddr.create8bit(0X76));
            rangeSensorLeftFront.setI2cAddress(I2cAddr.create8bit(0X28));
            rangeSensorLeftBack.setI2cAddress(I2cAddr.create8bit(0X26));

            rangeSensorRightFront.initialize();
            rangeSensorRightBack.initialize();
            rangeSensorLeftFront.initialize();
            rangeSensorLeftBack.initialize();


            rangeSensorBackLeft = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "range_sensor_back_left");
            rangeSensorBackRight = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "range_sensor_back_right");

            rangeSensorBackLeft.setI2cAddress(I2cAddr.create8bit(0X46));
            rangeSensorBackRight.setI2cAddress(I2cAddr.create8bit(0X48));

            rangeSensorBackLeft.initialize();
            rangeSensorBackRight.initialize();


            // get a reference to our ColorSensor object.
            //colorSensorLeft = hardwareMap.get(ColorSensor.class, "sensor_color_left");
            //colorSensorRight = hardwareMap.get(ColorSensor.class, "sensor_color_right");
            //colorSensorRight.setI2cAddress(I2cAddr.create8bit(0X3A));
        }
    }

    public void testSensors()
    {
        telemetry.addData("Right Sensors",(rangeSensorRightFront.getDistance(DistanceUnit.INCH) + rangeSensorRightBack.getDistance(DistanceUnit.INCH)) / 2 );
        telemetry.addData("Left Sensors",(rangeSensorLeftFront.getDistance(DistanceUnit.INCH) + rangeSensorLeftBack.getDistance(DistanceUnit.INCH)) / 2 );
        telemetry.addData("Back Sensors",(rangeSensorBackRight.getDistance(DistanceUnit.INCH) + rangeSensorBackLeft.getDistance(DistanceUnit.INCH)) / 2 );
        telemetry.addData("Gyro Sensor", gyro.getHeading());
        telemetry.addData("New Gyro Sensor", getNewGyroHeading() );

        telemetry.update();
    }

    /**
     * @param power power for the wheels
     * @param time time to drive in MILLISECONDS
     */
    public void strafeTime(double power, long time)
    {
        //set power to 'drive' motors
        moveOmni(0, power, 0);
        //wait for certain of time while motors are running
        //robotMecanum.wait(time);
        long setTime = System.currentTimeMillis();
        previousTime = opMode.getRuntime();

        while(System.currentTimeMillis() - setTime < (time))
        {
            moveOmni(0, power, gyroPID(180, opMode.getRuntime() - previousTime));
            previousTime = opMode.getRuntime();
        }
        //sets all power to zero afterwords
        moveOmni(0, 0, 0);
    }
    /**
     * @param power - sets power to wheels - negative power is backwards
     * @param time  - amount of time to run the motors in MILLISECONDS
     */
    public void driveTime(double power, long time)
    {
        //set power to 'drive' motors

        moveOmni(power, 0, 0);

        //wait for certain amount of time while motors are running
        //robotMecanum.wait(time);
        long setTime = System.currentTimeMillis();
        previousTime = opMode.getRuntime();

        while(System.currentTimeMillis() - setTime < (time))
        {
            moveOmni(power, 0, gyroPID(180, opMode.getRuntime() - previousTime));
            previousTime = opMode.getRuntime();
        }
        //sets all power to zero afterwords
        moveOmni(0, 0, 0);
    }
    /**
     *
     * @param delay - delay/wait time in SECONDS
     */
    public void wait(double delay)
    {
        while(!hasRun)
        {
            if(System.currentTimeMillis() - setTime > (delay* 1000))
            {
                hasRun = true;
            }
        }

        telemetry.addData("wait finished", "");
        telemetry.update();
    }
    public void initiateVuforia()
    {
        tensorFlow.initVuforia();
    }
    public void printGyroHeading()
    {
        telemetry.addData("Gyro Heading", gyro.getHeading() );
        telemetry.update();
    }
    //==============================================================================================   convertDistTicks
    //method takes in 2nd parameter for circumfrence of spinning object
    public int convertDistTicks(double distanceToTravel, double circumfrence)
    {
        //1440 revolutions = 1 rotation
        //1 rotation = 4

        double revolutions = distanceToTravel / circumfrence;
        int totalTicks = (int) Math.round(revolutions * TICK_PER_REVOLUTION / GEAR_RATIO);

        return totalTicks;
    }
    public void moveOmni(double drivePower, double strafePower, double rotatePower)
    {
        double drive = Math.signum(drivePower) * Math.pow(drivePower, 4);
        double strafe = Math.signum(-strafePower) * Math.pow(strafePower, 4);
        double rotate = rotatePower;

        double frontLeftPower = drive + strafe + rotate;
        double backLeftPower = drive - strafe + rotate;
        double frontRightPower = drive - strafe - rotate;
        double backRightPower = drive + strafe - rotate;

        //Set the wheel power according to variables
        frontLeftWheel.setPower(frontLeftPower);
        backLeftWheel.setPower(backLeftPower);
        frontRightWheel.setPower(frontRightPower);
        backRightWheel.setPower(backRightPower);

        //Print Telementary Data for the wheels
        /*telemetry.addData("frontLeftPower", frontLeftPower);
        telemetry.addData("backLeftPower", backLeftPower);
        telemetry.addData("frontRightPower", frontRightPower);
        telemetry.addData("backRightPower", backRightPower);*/
    }
    public void drive(double distance, double power)
    {
        setEncoders(distance);
        moveOmni( power, 0, 0);
        runEncoders(true,power);

    }

    public void strafe(double distance, double power)
    {
        setEncoders(distance);
        moveOmni( 0, power, 0);
        runEncoders(false,power);
    }

    private void setEncoders( double distance)
    {
        try
        {
            backRightWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backLeftWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontRightWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontLeftWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        catch(NullPointerException e)
        {
            telemetry.addData("Error" , "Cannot set \"RUN_USING_ENCODER\", check your mapping");
            telemetry.update();
        }

        try
        {
            backRightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backLeftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            frontRightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            frontLeftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        catch(Exception e)
        {
            telemetry.addData("Error" , "Cannot set \"STOP_AND_RESET_ENCODER\", check your mapping");
            telemetry.update();
        }
        setEncoderPos(convertDistTicks(distance, WHEEL_DIAMETER * Math.PI));

    }

    private boolean setEncoderPos(double distance)
    {

        boolean success = true;             // Tells whether setting encoders is successful or not

        /*
            The encoder counts needed for each wheel.
            When mecanum wheels move in an off-direction (angle not a multiple of 45),
            each wheel moves at a different speed. That's why we need separate variables.
         */
        int frontLeftEncoder;
        int backLeftEncoder;
        int frontRightEncoder;
        int backRightEncoder;

        double avgMotion;               // Average speed of the motors
        int avgCounts;                  // Average number of encoder counts to use

        try
        {
            /*
                With 45 degree rollers, the strafe distance on Mecanum wheels is the same
                as normal forward and backward distance(theoretically). Because the wheels move at
                different speeds, I opted to average them to find correct distance.
             */
            avgMotion = Math.abs((frontLeftWheel.getPower()) + Math.abs(backLeftWheel.getPower()) +
                    Math.abs(frontRightWheel.getPower()) + Math.abs(backRightWheel.getPower())) / 4;


            // Calculate the average number of counts for the motors.
            avgCounts = (int)(distance / (WHEEL_DIAMETER*Math.PI)* TICK_PER_REVOLUTION);


            // Set the individual counts.
            frontLeftEncoder = (int)(avgCounts * frontLeftWheel.getPower() / avgMotion);
            backLeftEncoder = (int)(avgCounts * backLeftWheel.getPower() / avgMotion);
            frontRightEncoder = (int)(avgCounts * frontRightWheel.getPower() / avgMotion);
            backRightEncoder = (int)(avgCounts * backRightWheel.getPower() / avgMotion);

            // Set the encoder values to the motors
            frontLeftWheel.setTargetPosition(frontLeftEncoder);
            backLeftWheel.setTargetPosition(backLeftEncoder);
            frontRightWheel.setTargetPosition(frontRightEncoder);
            backRightWheel.setTargetPosition(backRightEncoder);
        }
        catch(NullPointerException e)
        {
            telemetry.addData("Error" , "Cannot set encoder target positions, check your mapping");
            success = false;
        }

        return success;
    }

    /*boolean driveTo(final double distance , final double heading , final double speed)
    {
        // Create a new point to facilitate with cartesian/polar conversions
        UtilPoint myPoint = new UtilPoint(speed , heading , UtilPoint.Type.POLAR);

        boolean keepDriving = true;         // Determines whether or not to keep driving
        boolean success = true;             // Determines whether driving was successful or not


        setEncoders(distance);
        setEncoderPos(distance);

        // While any of the motors are doing things, keep setting power
        while(keepDriving)
        {
            if(Math.abs(frontLeftWheel.getCurrentPosition()) >= Math.abs(frontLeftWheel.getTargetPosition()) &&
                    Math.abs(backLeftWheel.getCurrentPosition()) >= Math.abs(backLeftWheel.getTargetPosition()) &&
                    Math.abs(frontRightWheel.getCurrentPosition()) >= Math.abs(frontRightWheel.getTargetPosition()) &&
                    Math.abs(backRightWheel.getCurrentPosition()) >= Math.abs(backRightWheel.getTargetPosition()))
                keepDriving = false;

            try
            {
                moveOmni(myPoint.y(),myPoint.x(),0);
            }
            catch(Exception e)
            {
                telemetry.addData("Error" , "Cannot power drivetrain, check your mapping");
                telemetry.update();
                success = false;
            }
        }

        return success;
    }*/


    private void runEncoders(boolean isDrive, double power)
    {
        backRightWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeftWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRightWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontLeftWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        boolean keepDriving = true;         // Determines whether or not to keep driving
        boolean success = true;             // Determines whether driving was successful or not
        previousTime = opMode.getRuntime();
        // While any of the motors are doing things, keep setting power
        while(keepDriving)
        {
            if(Math.abs(frontLeftWheel.getCurrentPosition()) >= Math.abs(frontLeftWheel.getTargetPosition()) &&
                    Math.abs(backLeftWheel.getCurrentPosition()) >= Math.abs(backLeftWheel.getTargetPosition()) &&
                    Math.abs(frontRightWheel.getCurrentPosition()) >= Math.abs(frontRightWheel.getTargetPosition()) &&
                    Math.abs(backRightWheel.getCurrentPosition()) >= Math.abs(backRightWheel.getTargetPosition()))
                keepDriving = false;

            try
            {
                if(isDrive)
                    moveOmni(power,0,gyroPID(180, opMode.getRuntime() - previousTime));
                else
                    moveOmni(0,power,gyroPID(180, opMode.getRuntime() - previousTime));
                previousTime = opMode.getRuntime();
            }
            catch(Exception e)
            {
                telemetry.addData("Error" , "Cannot power drivetrain, check your mapping");
                telemetry.update();
                success = false;
            }
        }

        // set motor power to zero to turn off motors. The motors stop on their own but
        // power is still applied so we turn off the power.
        moveOmni(0,0,0);


        backRightWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLeftWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    /**
     * + power and isRightSensor moves towards right wall
     * <br>- power and isRightSensor moves away from right wall
     * <br>+ power and !isRightSensor moves away from left wall
     * <br>- power and !isRightSensor moves towards left wall
     *
     * @param distanceFromWall distance to stop from the wall in inches
     * @param power power to run the motors. + moves right, - moves left
     * @param isRightSensor if the robot should use the right sensors
     *
     */
    public void strafeRange(double distanceFromWall, double power, boolean isRightSensor)
    {
        frontRightWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLeftWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        gyro.resetZAxisIntegrator();

        previousTime = opMode.getRuntime();

        //moveOmni(0, power, 0);

        // while moving toward the right wall OR moving away from the left wall OR moving away from the right wall OR moving toward the left wall
        while(     (power > 0 && isRightSensor  && (rangeSensorRightFront.getDistance(DistanceUnit.INCH) + rangeSensorRightBack.getDistance(DistanceUnit.INCH)) / 2 > distanceFromWall)
                || (power < 0 && !isRightSensor && ( rangeSensorLeftFront.getDistance(DistanceUnit.INCH) +  rangeSensorLeftBack.getDistance(DistanceUnit.INCH)) / 2 > distanceFromWall)      )
        {
            moveOmni(0, power, gyroPID(180, opMode.getRuntime() - previousTime));
            telemetry.addData("distance from wall", rangeSensorRightFront.getDistance(DistanceUnit.INCH));

            telemetry.addData("right positive", (rangeSensorRightFront.getDistance(DistanceUnit.INCH) + rangeSensorRightBack.getDistance(DistanceUnit.INCH)) / 2) ;
            telemetry.addData("left negative", (rangeSensorLeftFront.getDistance(DistanceUnit.INCH) + rangeSensorLeftBack.getDistance(DistanceUnit.INCH)) / 2 );
            telemetry.addData("right negative", (rangeSensorRightFront.getDistance(DistanceUnit.INCH) + rangeSensorRightBack.getDistance(DistanceUnit.INCH)) / 2 );
            telemetry.addData("left positive", (rangeSensorLeftFront.getDistance(DistanceUnit.INCH) + rangeSensorLeftBack.getDistance(DistanceUnit.INCH)) / 2 );
            telemetry.update();
            previousTime = opMode.getRuntime();
        }

        telemetry.addData("away", (power > 0  && !isRightSensor && (rangeSensorLeftFront.getDistance(DistanceUnit.INCH) + rangeSensorLeftBack.getDistance(DistanceUnit.INCH)) / 2 < distanceFromWall)
                                             || (power < 0  && isRightSensor  && (rangeSensorRightFront.getDistance(DistanceUnit.INCH) + rangeSensorRightBack.getDistance(DistanceUnit.INCH)) / 2 < distanceFromWall) );
        telemetry.update();

        //moving towards a wall
        while((power > 0 && !isRightSensor && ( rangeSensorLeftFront.getDistance(DistanceUnit.INCH) +  rangeSensorLeftBack.getDistance(DistanceUnit.INCH)) / 2 < distanceFromWall)
           || (power < 0 && isRightSensor  && (rangeSensorRightFront.getDistance(DistanceUnit.INCH) + rangeSensorRightBack.getDistance(DistanceUnit.INCH)) / 2 < distanceFromWall)  )
        {
            moveOmni(0, power, rangePID(0,rangeSensorBackLeft.getDistance(DistanceUnit.INCH), rangeSensorBackRight.getDistance(DistanceUnit.INCH)  , opMode.getRuntime() - previousTime) );
            telemetry.addData("distance from wall", rangeSensorRightFront.getDistance(DistanceUnit.INCH));

            telemetry.addData("right positive", (rangeSensorRightFront.getDistance(DistanceUnit.INCH) + rangeSensorRightBack.getDistance(DistanceUnit.INCH)) / 2) ;
            telemetry.addData("left negative", (rangeSensorLeftFront.getDistance(DistanceUnit.INCH) + rangeSensorLeftBack.getDistance(DistanceUnit.INCH)) / 2 );
            telemetry.update();
            previousTime = opMode.getRuntime();
        }
        moveOmni(0,0,0);
    }


    /**
     * Strafes toward a wall using ultrasonic sensors
     * @param power power at which to strafe toward wall
     * @param distanceFromWall distance at which to move to relative to the wall
     * @param usingRightSensors if the right sensors should be used
     */
    public void strafeTowardWall(double power, int distanceFromWall, boolean usingRightSensors){
        frontRightWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLeftWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        gyro.resetZAxisIntegrator();

        previousTime = opMode.getRuntime();

        power = usingRightSensors? power : -power;
        moveOmni(0, power, 0);

        while(((LinearOpMode) opMode).opModeIsActive() && (usingRightSensors && (rangeSensorRightFront.getDistance(DistanceUnit.INCH) + rangeSensorRightBack.getDistance(DistanceUnit.INCH)) / 2 > distanceFromWall)
        || (!usingRightSensors && (rangeSensorLeftFront.getDistance(DistanceUnit.INCH) +  rangeSensorLeftBack.getDistance(DistanceUnit.INCH)) / 2 > distanceFromWall)) {
            moveOmni(0, power, gyroPID(180, opMode.getRuntime() - previousTime));
            previousTime = opMode.getRuntime();
            telemetry.addData("rightT", (rangeSensorRightFront.getDistance(DistanceUnit.INCH) + rangeSensorRightBack.getDistance(DistanceUnit.INCH)) / 2) ;
            telemetry.addData("leftT", (rangeSensorLeftFront.getDistance(DistanceUnit.INCH) + rangeSensorLeftBack.getDistance(DistanceUnit.INCH)) / 2 );
            telemetry.update();
        }
        moveOmni(0,0,0);
    }

    /**
     * Strafes away from a wall using ultrasonic sensors
     * @param power power at which to strafe away from wall
     * @param distanceFromWall distance at which to move to relative to the wall
     * @param usingRightSensors if the right sensors should be used
     */
    public void strafeAwayFromWall(double power, int distanceFromWall, boolean usingRightSensors)
    {
        frontRightWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLeftWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        gyro.resetZAxisIntegrator();

        previousTime = opMode.getRuntime();

        power = usingRightSensors? -power : power;
        moveOmni(0, power, 0);

        while((usingRightSensors && rangeSensorRightFront.getDistance(DistanceUnit.INCH) < distanceFromWall)
          || (!usingRightSensors && rangeSensorLeftFront.getDistance(DistanceUnit.INCH) < distanceFromWall))
            {
            moveOmni(0, power, gyroPID(180, opMode.getRuntime() - previousTime));
            previousTime = opMode.getRuntime();
            telemetry.addData("rightA", (rangeSensorRightFront.getDistance(DistanceUnit.INCH) + rangeSensorRightBack.getDistance(DistanceUnit.INCH)) / 2) ;
            telemetry.addData("leftA", (rangeSensorLeftFront.getDistance(DistanceUnit.INCH) + rangeSensorLeftBack.getDistance(DistanceUnit.INCH)) / 2 );
            telemetry.update();
        }
        moveOmni(0,0,0);
    }
    public void driveRange(double distanceFromWall, double power)
        {
        frontRightWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLeftWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        gyro.resetZAxisIntegrator();

        previousTime = opMode.getRuntime();

        moveOmni(power, 0, 0);

        // while moving toward the right wall OR moving away from the left wall OR moving away from the right wall OR moving toward the left wall
        while((power > 0 && (rangeSensorBackRight.getDistance(DistanceUnit.INCH) + rangeSensorBackLeft.getDistance(DistanceUnit.INCH)) / 2 < distanceFromWall)
         ||   (power < 0 && (rangeSensorBackRight.getDistance(DistanceUnit.INCH) + rangeSensorBackLeft.getDistance(DistanceUnit.INCH)) / 2 > distanceFromWall) )
        {
            moveOmni(power, 0, gyroPID(180, opMode.getRuntime() - previousTime));
            telemetry.addData("distance from wall", rangeSensorBackRight.getDistance(DistanceUnit.INCH));
            telemetry.update();
            previousTime = opMode.getRuntime();
        }
        moveOmni(0,0,0);
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
    public double convertDegreeToPercent(double desiredAngle, double maxAngle)
    {
        return desiredAngle/maxAngle;
    }
    public double gyroPID(int targetAngle, double time)
    {
        int error = targetAngle - getNewGyroHeading(); // Error = Target - Actual
        this.integral += (error * time); // Integral is increased by the error*time
        double derivative = (error - this.previous_error) / time;
        telemetry.addData("PID correction value:", P * error + I*this.integral);
        telemetry.update();
        return P * error + I * this.integral + D * derivative;
    }

    /**
     * @param targetAngleFromWall   - angle from the wall - 0 is parallel
     * @param range1Reading         - the distance from back left sensor to the wall    doesn't matter which you put in, left or right sensor
     * @param range2Reading         - the distance from back right sensor to the wall                           ||
     * @param time                  - current time  -  usually (opMode.getRuntime() - previousTime)
     */
    public double rangePID(int targetAngleFromWall, double range1Reading, double range2Reading, double time)
    {
        double error = targetAngleFromWall - Math.atan( (Math.abs(range1Reading-range2Reading))/BACK_SENSOR_SEPERATION ); // Error = Target - Actual
        this.integral += (error * time); // Integral is increased by the error*time
        double derivative = (error - this.previous_error) / time;
        telemetry.addData("PID correction value:", P * error);
        telemetry.update();
        return P * error + I * this.integral + D * derivative;
    }
    public int getNewGyroHeading()
    {
        return (gyro.getHeading() + 180 ) % 360;
    }
    //==============================================================================================   turnGyro
    //not ready or programmed yet
    public void turnGyro(double turningDegrees, double power, boolean turnRight)
    {

        telemetry.addData("turnGyro", "running");

        frontRightWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLeftWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        gyro.resetZAxisIntegrator();
        if(turnRight)
        {
            //mostly works --- turned right 270 when I wanted it to move 90 right at 0.7 power
            moveOmni(0, 0, -power);
            while( getNewGyroHeading() > 180 - turningDegrees)
            {
                telemetry.addData("New While:", getNewGyroHeading() > 180 - turningDegrees);
                telemetry.addData("Heading + 180:", getNewGyroHeading());
                telemetry.update();
            }
            moveOmni(0, 0, 0);
        }
        //doesn't work --- had to leave didn't get to finishing this up.
        else
        {
            moveOmni(0, 0, power);
            while( getNewGyroHeading() < 180 + turningDegrees)
            {
                telemetry.addData("While:", getNewGyroHeading() < 180 + turningDegrees);
                telemetry.addData("Heading + 180:", getNewGyroHeading());
                telemetry.update();
            }
            moveOmni(0, 0, 0);
        }
    }
    public void capper(boolean capperHome)
    {
        if (capperHome)
        {
            capperPosition = CAPPER_HOME;
        }
        else
        {
            capperPosition = CAPPER_EXTENDED;
        }

        capper.setPosition(capperPosition);

        telemetry.addData("Capper Position: ", capper.getPosition());
        telemetry.update();
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
    public void shuffle(double power, double distance)
    {
        for(int i = 0; i < 3; i++)
            drive(distance, power);
        for(int i = 0; i < 3; i++)
            drive(-distance, power);
    }
    public void tensorFlowPrep()
    {
        do
        {
            tensorFlow.tensorFlow();
            //shuffle(0.2, 1);
        }while(tensorFlow.needsShuffle);

        if (tensorFlow.getSkystonePosition() == TensorFlow.Position.none)
            skystoneLocation = "none";
        else if (tensorFlow.getSkystonePosition() == TensorFlow.Position.left)
            skystoneLocation = "left";
        else if (tensorFlow.getSkystonePosition() == TensorFlow.Position.right)
            skystoneLocation = "right";
        else
            telemetry.addData("Error: ", "No Move");
        telemetry.update();
    }

    /**
     * Runs the lift to a certain position specified
     * by how many blocks high the lift needs to go
     *
     * @param numBlocks how many blocks to move up
     * @param power power at which to run the lift
     */
    public void setLiftPosition(int numBlocks, double power){
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        lift.setTargetPosition(numBlocks * TICKS_PER_BLOCK);

        lift.setPower(power);

        while(lift.isBusy());

        lift.setPower(0);

        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

}

