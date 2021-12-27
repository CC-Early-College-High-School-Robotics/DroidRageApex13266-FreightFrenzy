package org.firstinspires.ftc.teamcode.teleop.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
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
        RevColorSensorV3 colorSensorV3;


        colorSensorV3 = hardwareMap.get(RevColorSensorV3.class, colorSensorName);


        waitForStart();
//        pos = servo.getPosition();
        while(!isStopRequested()) {

//            servo.setPosition(pos);
//            telemetry.addData("servo pos", servo.getPosition());
            telemetry.addData("desired pos", colorSensorV3.argb());
            telemetry.update();
            sleep(10);
        }
    }
}
