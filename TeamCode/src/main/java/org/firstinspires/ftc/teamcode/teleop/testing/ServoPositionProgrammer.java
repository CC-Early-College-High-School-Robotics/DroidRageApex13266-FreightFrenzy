package org.firstinspires.ftc.teamcode.teleop.testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp(name="Servo Position Programmer", group="test")
public class ServoPositionProgrammer extends LinearOpMode {
    public static double pos;
    public static String servoName = "boxServo";

    @Override
    public void runOpMode() {

        //hardware initialization
        Servo servo;


        servo = hardwareMap.get(Servo.class, servoName);


        waitForStart();
        pos = servo.getPosition();
        while(!isStopRequested()) {


            if (gamepad1.a) {
                pos -= 0.001;
            } else if (gamepad1.b) {
                pos += 0.001;
            }
            servo.setPosition(pos);
            telemetry.addData("servo pos", servo.getPosition());
            telemetry.addData("desired pos", pos);
            telemetry.update();
            sleep(10);
        }
    }
}
