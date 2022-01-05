package org.firstinspires.ftc.teamcode.teleop.testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp(name="ControllerCircleTest", group="test")
public class ControllerCircleTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        telemetry.addData("controller angle degrees:", Math.toDegrees(Math.atan2(-gamepad2.left_stick_y, gamepad1.left_stick_x)));
        telemetry.update();
        sleep(10);
    }
}

