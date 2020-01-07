
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

@Autonomous(name="Autonomous-FoundationBlue")
//@disabled
public class AutonomousFoundationBlue extends LinearOpMode
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

        sideFoundation(-1, 750, 1200, 400, 80, 500, 1750);
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
        robotMecanum.liftTime(0.5, 1000);

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

        robotMecanum.gyro.resetZAxisIntegrator();
        if(isRedField == 1)
        {
            while (robotMecanum.getNewGyroHeading() < 180 + turnAngle)
                robotMecanum.moveMotors(leftMove, leftMove, rightMove, rightMove);
        }
        if(isRedField == -1)
        {
            while (robotMecanum.getNewGyroHeading() > 180 - turnAngle)
                robotMecanum.moveMotors(leftMove, leftMove, rightMove, rightMove);
        }
        //robotMecanum.moveOmni(0.5,0, 0.5 * isRedField)

        robotMecanum.omniTime(0.7, 0, drive3Time);
        robotMecanum.hooks(true);
        robotMecanum.omniTime(-0.65, 0, drive4Time);
    }

}

