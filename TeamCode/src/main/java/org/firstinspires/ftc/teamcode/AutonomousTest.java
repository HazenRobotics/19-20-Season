
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
        {
            robotMecanum.testSensors(false);
        }
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

       // robotMecanum.strafeRange(14.5, 0.75, true);

    /*
        //parking(true, false, 3);
        robotMecanum.drive(20, 0.75);
        robotMecanum.drive(-20, 0.75);
    */

        sideFoundation(-1, 750, 1200, 400, 80, 250, 1500);
        //robotMecanum.turnGyro(90,0.2,true);
    }

    /**
     *drives and picks up foundation and moves foundation to target zone <br><br> <b>Note: time is in ms</b>
     *
     * @param isRedField 1 if red and -1 if blue
     * @param strafeTime time to align with middle of foundation <i>Target: 11"</i>
     * @param drive1Time time to drive to foundation      <i>Target: 30"</i>
     * @param drive2Time time to drive foundation back    <i>Target: 6"</i>
     * @param turnAngle angle need to turn foundation against wall
     * @param drive3Time time to drive foundation against wall <i>Target: 6"</i>
     * @param drive4Time time to drive to park
     */
    public void sideFoundation(int isRedField, int strafeTime, int drive1Time, int drive2Time, int turnAngle, int drive3Time, int drive4Time)
    {
        //double leftMove = 0, rightMove = 0;
        //robotMecanum.strafeRange(16, 0.75, true);

        //driveTime(0.65, 900);
        robotMecanum.strafeTime(-isRedField * 0.75, strafeTime);
        sleep(250);
        robotMecanum.driveTime(0.7, drive1Time);
        sleep(250);
        //robotMecanum.driveRange(34, 0.7);
        robotMecanum.hooks(false);
        sleep(500);
        robotMecanum.driveTime(-0.75, drive2Time);
        sleep(250);

       /* if(isRedField == -1)
            rightMove = -0.65;
        else
            leftMove = -0.65;*/

        double rightMove = isRedField == -1 ? 0.65 : 0;
        double leftMove = isRedField == -1 ? 0 : 0.65;
        while ((robotMecanum.getNewGyroHeading() <= 180 + turnAngle && isRedField == -1) || (robotMecanum.getNewGyroHeading() >= 180 - turnAngle && isRedField == 1))
            robotMecanum.moveMotors(leftMove, leftMove, rightMove, rightMove);
        //robotMecanum.moveOmni(0.5,0, 0.5 * isRedField)

        robotMecanum.driveTime(0.75, drive3Time);

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

