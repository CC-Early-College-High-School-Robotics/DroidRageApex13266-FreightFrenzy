package org.firstinspires.ftc.teamcode.teleop.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.kotlin.extensions.gamepad.GamepadExExtKt;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.internal.ui.GamepadUser;

import java.awt.font.NumericShaper;

@Config
@TeleOp(name="ControllerCircleTest", group="test")
public class ControllerCircleTest extends LinearOpMode {


    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        GamepadEx hi = new GamepadEx(gamepad1);

        waitForStart();
//        pos = servo.getPosition();
        while(!isStopRequested()) {
            telemetry.addData("controller 0.8 deadzone test", Range.clip(Math.sqrt((Math.pow(-gamepad1.left_stick_y, 2)) + (Math.pow(gamepad1.left_stick_x, 2))), 0.8, 1));
            telemetry.addData("controller distance formula test", Math.sqrt((Math.pow(-gamepad1.left_stick_y, 2)) + (Math.pow(gamepad1.left_stick_x, 2))));
            telemetry.addData("controller angle degrees:", Math.toDegrees(Math.atan2(gamepad1.left_stick_x, -gamepad1.left_stick_y)));
            telemetry.addData("controller angle degrees with 45 extra and range clipped divided by 270", (Range.clip(Math.toDegrees(Math.atan2(gamepad1.left_stick_x, -gamepad1.left_stick_y)) + 135, 0, 270)) / 270);
            telemetry.addData("controller angle degrees with 45 extra and range clipped", Range.clip(Math.toDegrees(Math.atan2(gamepad1.left_stick_x, -gamepad1.left_stick_y)) + 135, 0, 270));
            telemetry.addData("controller y", -gamepad1.left_stick_y);
            telemetry.addData("controller x", gamepad1.left_stick_x);
            telemetry.update();
            sleep(10);
        }
    }
}

