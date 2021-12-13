package org.firstinspires.ftc.teamcode.hardware.subsystembase.main;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.Devices;
import org.firstinspires.ftc.teamcode.hardware.subsystembase.base.BaseSubsystem;

@Config
public class BoxSubsystem extends BaseSubsystem {
    // Values
    public static double BOX_UP = 0.641;
    public static double BOX_AUTO_APPROACH_HUB = 0.500;
    public static double BOX_INTAKE = 0.925;
    public static double BOX_DROP = 0.180;

    // Create hardware variables
    public Servo boxServo = null;

    // Constructor
    public BoxSubsystem(Gamepad gamepad1, Gamepad gamepad2, HardwareMap hardwareMap, Telemetry telemetry) {
        super(gamepad1, gamepad2, hardwareMap, telemetry);
    }

    // Initialize hardware variables
    public void init() {
        boxServo = hardwareMap.get(Servo.class,"boxServo");
        boxServo.setDirection(Servo.Direction.FORWARD);
    }

    // Default command
    public void defaultCommand() {
        // High
        if (gamepad2.dpad_up) {
            boxServo.setPosition(BOX_UP);
        }
        // Mid
        if (gamepad2.dpad_right) {
            boxServo.setPosition(BOX_UP);
        }
        // Shared hub
        if (gamepad2.dpad_down) {
            boxServo.setPosition(BOX_UP);

        }
        // intake/reset position
        if (gamepad2.dpad_left) {
            boxServo.setPosition(BOX_INTAKE);
        }
        if (gamepad2.a) {
            boxServo.setPosition(Devices.BOX_INTAKE);
        }
        if (gamepad2.x) {
            boxServo.setPosition(Devices.BOX_DROP);
        }
        if (gamepad2.y) {
            boxServo.setPosition(Devices.BOX_UP);
        }
    }
}