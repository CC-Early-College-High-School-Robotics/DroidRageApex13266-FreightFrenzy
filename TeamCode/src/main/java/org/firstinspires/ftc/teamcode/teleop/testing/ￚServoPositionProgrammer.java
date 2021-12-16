package org.firstinspires.ftc.teamcode.teleop.testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp(name="Servo Position Programmer", group="test")
public class ￚServoPositionProgrammer extends LinearOpMode {
    public static double pos;
    String mode = "passive";
    boolean xPressed = false;
    public static String servoName = "boxServo";

    @Override
    public void runOpMode() {
        TuningStart.initializeTuning();

        //hardware initialization
        Servo servo;


        servo = hardwareMap.get(Servo.class, servoName);

        pos = servo.getPosition();
        sleep(1000);
        waitForStart();
        while(!isStopRequested()) {
            if (gamepad1.x  && !xPressed && mode.equals("passive")) {
                mode = "active";
                xPressed = true;
            }
            if (gamepad1.x  && !xPressed && mode.equals("active")) {
                mode = "passive";
                xPressed = true;
            }
            if (!gamepad1.x && xPressed) {
                xPressed = false;
            }

            if (mode.equals("active")) {
                pos = servo.getPosition();
            }

            if (gamepad1.a && mode.equals("active")) {
                pos -= 0.001;
            } else if (gamepad1.b && mode.equals("active")) {
                pos += 0.001;
            }

            servo.setPosition(pos);
            telemetry.addLine("Press x to switch between passive get position and active set position modes");
            telemetry.addData("Current mode", mode);
            telemetry.addLine();
            telemetry.addData("Current Servo pos", servo.getPosition());

            if (mode.equals("active")) {
                telemetry.addData("Desired Servo pos", pos);
            }

            telemetry.update();
            sleep(10);
        }
    }
}
