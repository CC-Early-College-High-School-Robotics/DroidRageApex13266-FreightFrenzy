package org.firstinspires.ftc.teamcode.teleop.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp(name="Color Sensor Programmer", group="test")
public class ￚColorSensorProgrammer extends LinearOpMode {
    public static double pos;
    public static String colorSensorName = "boxSensor";

    @Override
    public void runOpMode() {
        TuningStart.initializeTuning();

        //hardware initialization
//        RevColorSensorV3 colorSensorV3;
        ColorSensor colorSensor;


//        colorSensorV3 = hardwareMap.get(RevColorSensorV3.class, colorSensorName);
        colorSensor = hardwareMap.get(ColorSensor.class, colorSensorName);


        waitForStart();
//        pos = servo.getPosition();
        while(!isStopRequested()) {

//            servo.setPosition(pos);
//            telemetry.addData("servo pos", servo.getPosition());

            telemetry.addData("Servo alpha", colorSensor.alpha());
            telemetry.addData("Servo red", colorSensor.red());
            telemetry.addData("Servo blue", colorSensor.blue());
            telemetry.addData("Servo green", colorSensor.green());
            telemetry.update();
            sleep(10);
        }
    }
}
