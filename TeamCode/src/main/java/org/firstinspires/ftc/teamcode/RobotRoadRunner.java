package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.ArrayList;
import java.util.List;

//TODO: Rewrite based on sample RR Mechanum drive class
public class RobotRoadRunner {
    PIDFController controller;
    PIDCoefficients coeff;
    MecanumDrive drive;
    RobotMecanum robotMecanum;

    public RobotRoadRunner(HardwareMap hMap, OpMode opMode){
        robotMecanum = new RobotMecanum(hMap, opMode, false);
        drive = new MecanumDrive(0, 0, 0, 0, 0) {
            @Override
            public void setMotorPowers(double v, double v1, double v2, double v3) {
                robotMecanum.moveMotors(v, v1, v3, v2);
            }

            @Override
            public List<Double> getWheelPositions() {
                List<Double> positions = new ArrayList<>();
                positions.add(robotMecanum.convertTicksDist(robotMecanum.frontLeftWheel.getCurrentPosition()));
                positions.add(robotMecanum.convertTicksDist(robotMecanum.backLeftWheel.getCurrentPosition()));
                positions.add(robotMecanum.convertTicksDist(robotMecanum.backRightWheel.getCurrentPosition()));
                positions.add(robotMecanum.convertTicksDist(robotMecanum.frontRightWheel.getCurrentPosition()));
                return positions;
            }

            @Override
            protected double getRawExternalHeading() {
                return robotMecanum.gyro.getHeading();
            }
        };
    }
}
