package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import java.util.*;

import java.util.List;

public class RoadRunnerRed extends LinearOpMode {

    @Override
    public void runOpMode(){
        final RobotMecanum robotMecanum = new RobotMecanum(hardwareMap, this, false);
        MecanumDrive robot = new MecanumDrive(0, 0, 0, 0) {
            @Override
            public void setMotorPowers(double v, double v1, double v2, double v3) {
                robotMecanum.moveMotors(v, v1, v3, v2);
            }

            @Override
            public List<Double> getWheelPositions() {
                return new ArrayList<Double>() {{
                    add((double)robotMecanum.frontLeftWheel.getCurrentPosition());
                    add((double)robotMecanum.backLeftWheel.getTargetPosition());
                    add((double)robotMecanum.backRightWheel.getCurrentPosition());
                    add((double)robotMecanum.frontRightWheel.getCurrentPosition()); }};
            }

            @Override
            protected double getRawExternalHeading() {
                return robotMecanum.gyro.getHeading() * Math.PI / 180;
            }
        };
    }
}
