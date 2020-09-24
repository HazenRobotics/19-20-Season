package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;

@TeleOp(name="UltrasonicTest")
public class UltrasonicTest extends LinearOpMode {

    UltrasonicSensor rangeSensor1;
    //UltrasonicSensor rangeSensor2;
    //UltrasonicSensor rangeSensor3;

    @Override
    public void runOpMode() throws InterruptedException {
        rangeSensor1 = hardwareMap.ultrasonicSensor.get("ultrasonic_sensor");
        //rangeSensor2 = hardwareMap.ultrasonicSensor.get("range_sensor_2");
        //rangeSensor3 = hardwareMap.ultrasonicSensor.get("range_sensor_3");

        waitForStart();

        while(opModeIsActive()){
            telemetry.addData("Range 1", rangeSensor1.getUltrasonicLevel());
            //telemetry.addData("Range 2", rangeSensor2.getUltrasonicLevel());
            //telemetry.addData("Range 3", rangeSensor3.getUltrasonicLevel());
            telemetry.update();
        }
    }
}
