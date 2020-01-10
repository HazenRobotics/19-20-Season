
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

// autonomous program that drives bot forward a set distance, stops then
// backs up to the starting point using encoders to measure the distance.
// This example assumes there is one encoder, attached to the left motor

@Autonomous(name="AutonomousTest")
//@disabled
public class AutonomousTest extends LinearOpMode
{
    RobotMecanum robotMecanum;

    @Override
    public void runOpMode() throws InterruptedException
    {
        robotMecanum = new RobotMecanum(hardwareMap, this, false);

        //robotMecanum.initiateVuforia();

        robotMecanum.gyro.calibrate();

        //==========================================================================================
        //Pre init

        robotMecanum.hooks(true);
        robotMecanum.claw(true);

        telemetry.addData("Step 1", "init finished");
        telemetry.update();

        waitForStart();

        //==========================================================================================
        //Official Start
/*
        while(opModeIsActive())
            robotMecanum.testSensors(false);
*/

        /*
        //robotMecanum.strafeTowardWall(0.75,16,false);
        telemetry.setAutoClear(false);
        //robotMecanum.strafe(-10,0.75);
        //robotMecanum.drive(-10,0.75);
        //robotMecanum.strafeAwayFromWall(0.75, 40, false);

        //robotMecanum.strafeTime(0.75,2000);
        int times = 2;
        for(int i = 0; i < times; i++)
        {
            sleep(10 * 1000);
            robotMecanum.strafeRange(50, 0.75, false);

            sleep(10 * 1000);
            robotMecanum.strafeRange(50, -0.75, true);

        }
        */

        /*
            //step one, move forward to scan blocks
            robotMecanum.driveTime(0.65, 1000);
            robotMecanum.driveRange(14, 0.75);


            //scan blocks ands organize information
            robotMecanum.tensorFlowPrep();
            sleep(5000);

            //tensor flow driving to blocks
            tensorFlowDrive();
    */

        //return back to center
            //skystoneReturn();

            //robotMecanum.strafeRange(14.5, 0.75, true);

        /*
            //parking(true, false, 3);
            robotMecanum.drive(20, 0.75);
            robotMecanum.drive(-20, 0.75);
        */

        sideFoundation(1, 750, 1200, 500, 2000, 2000, 500, 1000, 500);

        //robotMecanum.turnGyro(90,0.2,true);
    }

    /**
     * @param isRedField -1 means you are on the red field
     * @param driveForwardTime1 the amount of time to drive forward to the first block
     * @param driveBackTime the amount of time to move back to prepare to strafe
     * @param strafeFoundationTime1 amount of time to strafe to the other side
     * @param strafeBrickTime1 amount of time to strafe back to the bricks
     * @param driveForwardTime2 amount of time to drive forward to the second/third block
     * @param strafeFoundationTime2 amount of time to strafe to the other side
     * @param strafeBrickTime2 amount of time to strafe back to the midline
     */
    public void sideBricks(int isRedField, int driveForwardTime1, int driveBackTime, int strafeFoundationTime1, int strafeBrickTime1, int driveForwardTime2, int strafeFoundationTime2, int strafeBrickTime2)
    {
        sleep(4000000);
        //isRedField is -1 for
        robotMecanum.omniTime(0.7, 0, driveForwardTime1);

        claw("extended");
        sleep(2300);
        robotMecanum.omniTime(-0.6, 0, driveBackTime);

        robotMecanum.omniTime(0, 0.7 * -isRedField, strafeFoundationTime1);

        claw("home");
        sleep(500);
        robotMecanum.omniTime(0, 0.7 * isRedField, strafeBrickTime1);

        robotMecanum.omniTime(0.7, 0, driveForwardTime2);

        claw("extended");
        sleep(2300);
        robotMecanum.omniTime(0.6, 0, driveForwardTime2);

        robotMecanum.omniTime(-0.6, 0, driveBackTime);

        robotMecanum.omniTime(0, 0.7 * -isRedField, strafeFoundationTime2);

        claw("home");
        sleep(500);
        robotMecanum.omniTime(0, 0.7 * isRedField, strafeBrickTime2);


    }

    public void claw(String clawHome)
    {
        if(clawHome.equalsIgnoreCase("home"))
            robotMecanum.claw(true);
        if(clawHome.equalsIgnoreCase("extended"))
            robotMecanum.claw(false);
    }


    /**
     *drives and picks up foundation and moves foundation to target zone <br><br> <b>Note: time is in ms</b>
     *
     * @param isRedField -1 if red and 1 if blue
     * @param strafeTime time to align with middle of foundation <i>Target: 11"</i>
     * @param drive1Time time to drive to foundation      <i>Target: 30"</i>
     * @param drive2Time time to drive foundation back    <i>Target: 6"</i>
     * @param turnTime time to turn foundation against wall
     * @param drive3Time time to drive foundation against wall <i>Target: 6"</i>
     */
    public void sideFoundation(int isRedField, int strafeTime, int drive1Time, int drive2Time, int turnTime, int drive3Time, int strafe2Time, int driveBackTime, int driveBackTime2)
    {
        robotMecanum.setLiftPosition(1,0.9);

        //double leftMove = 0, rightMove = 0;
        //robotMecanum.strafeRange(16, 0.75, true);

        //driveTime(0.65, 900);
        robotMecanum.omniTime(0, -isRedField * 0.75, strafeTime);
        sleep(250);
        robotMecanum.omniTime(0.7, 0, drive1Time);

        sleep(250);
        //robotMecanum.driveRange(34, 0.7);
        robotMecanum.hooks(false);
        sleep(500);
        robotMecanum.omniTime(-0.75, 0, drive2Time);
        sleep(250);

       /* if(isRedField == -1)
            rightMove = -0.65;
        else
            leftMove = -0.65;*/

        double rightMove = isRedField == -1 ? 0.65 : 0;
        double leftMove = isRedField == -1 ? 0 : 0.65;

        if(isRedField == 1)
        {
            robotMecanum.moveMotors(leftMove, leftMove, rightMove, rightMove);
            sleep(turnTime);
        }
        if(isRedField == -1)
        {
            robotMecanum.moveMotors(leftMove, leftMove, rightMove, rightMove);
            sleep(turnTime);
        }
        robotMecanum.moveOmni(0, 0, 0);

        robotMecanum.omniTime(0.75, 0, drive3Time);
        //robotMecanum.gyro.resetZAxisIntegrator();
        /*if(isRedField == 1)
        {
            while (robotMecanum.getNewGyroHeading() < 180 + turnAngle)
                robotMecanum.moveMotors(leftMove, leftMove, rightMove, rightMove);
        }
        if(isRedField == -1)
        {
            while (robotMecanum.getNewGyroHeading() > 180 - turnAngle)
                robotMecanum.moveMotors(leftMove, leftMove, rightMove, rightMove);
        }*/
        //robotMecanum.moveOmni(0.5,0, 0.5 * isRedField)

        //robotMecanum.omniTime(0.7, 0, drive3Time);

        robotMecanum.hooks(true);

        robotMecanum.omniTime(0, -isRedField*0.75, strafe2Time);

        robotMecanum.omniTime(-0.75, 0, driveBackTime);

        robotMecanum.setLiftPosition(0,0.75);

        robotMecanum.omniTime(-0.75,0, driveBackTime2);
    }

    /*
    private void sideFoundationRange(int isRedField)
    {
        robotMecanum.strafeTowardWall(-isRedField * 0.75, 22, true);
        sleep(250);
        robotMecanum.driveRange(30, 0.7);
        sleep(250);
        //robotMecanum.driveRange(34, 0.7);
        robotMecanum.hooks(false);
        sleep(500);
        robotMecanum.driveTime(-0.75, drive2Time);
        sleep(250);
*/
       /* if(isRedField == -1)
            rightMove = -0.65;
        else
            leftMove = -0.65;*/
/*
        double rightMove = isRedField == -1 ? -0.65 : 0;
        double leftMove = isRedField == -1 ? 0 : -0.65;
        while ((robotMecanum.getNewGyroHeading() <= 180 + turnAngle && isRedField == -1) || (robotMecanum.getNewGyroHeading() >= 180 - turnAngle && isRedField == 1))
            robotMecanum.moveMotors(leftMove, leftMove, rightMove, rightMove);
    }
    */

    //==============================================================================================
    /**
     * @param isParkingLeft - parking to the left of robot
     * @param isParkingFar  - parking far from starting wall
     * @param sleepTime     - length of time to wait/sleep in SECONDS
     */
    public void parking(boolean isParkingLeft, boolean isParkingFar, int sleepTime)
    {
        sleep(sleepTime * 1000);

        if(isParkingFar)
        {
            robotMecanum.drive(18, 0.75);
        }
        if(isParkingLeft)
        {
            //robotMecanum.strafeRange(65, -0.75, true);
        }
        else
        {
            //robotMecanum.strafeRange(65, 0.75, false);
        }
    }
    public void skystoneReturn()
    {
        //robotMecanum.strafeRange(74, 0.75, false);
    }

    public void skystoneNone()
    {
        //strafe over to block
        //robotMecanum.strafeRange(14, -0.75, false);

        //drive forward to pickup block
        robotMecanum.drive(12, 0.75);

        //pickup block
        robotMecanum.claw(true);

        //drive backward
        robotMecanum.drive(-12, 0.75);

        telemetry.addData("skystoneNone", "completed");
        telemetry.update();
    }

    public void skystoneLeft()
    {
        //strafe over to block
        //robotMecanum.strafeRange(22, -0.75, false);

        //drive forward to pickup block
        robotMecanum.drive(12, 0.75);

        //pickup block
        robotMecanum.claw(true);

        //drive backward
        robotMecanum.drive(-12, 0.75);

        telemetry.addData("skystoneNone", "completed");
        telemetry.update();
    }

    public void skystoneRight()
    {
        //strafe over to block
        //robotMecanum.strafeRange(30, -0.75, false);

        //drive forward to pickup block
        robotMecanum.drive(12, 0.75);

        //pickup block
        robotMecanum.claw(true);

        //drive backward
        robotMecanum.drive(-12, 0.75);

        telemetry.addData("skystoneNone", "completed");
        telemetry.update();
    }

    public void tensorFlowDrive()
    {
        if ( robotMecanum.skystoneLocation.equals("none") )
        {
            telemetry.addData("move to the skystone offscreen", "");
            telemetry.update();
            skystoneNone();
        }
        else if ( robotMecanum.skystoneLocation.equals("left") )
        {
            telemetry.addData("move to the left skystone position", "");
            telemetry.update();
            skystoneLeft();
        }
        else if ( robotMecanum.skystoneLocation.equals("right") )
        {
            telemetry.addData("move to the right skystone position", "");
            telemetry.update();
            skystoneRight();
        }
        else
            telemetry.addData("Error: ", "No Move");
        telemetry.update();
    }


}

