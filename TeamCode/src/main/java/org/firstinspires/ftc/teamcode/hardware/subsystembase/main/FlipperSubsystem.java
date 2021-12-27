package org.firstinspires.ftc.teamcode.hardware.subsystembase.main;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.subsystembase.base.BaseSubsystem;

@Config
@Disabled
public class FlipperSubsystem extends BaseSubsystem {
    // Values
    public static double FLIPPER_OPEN = 1;
    public static double FLIPPER_CLOSED = 0;

    // Create hardware variables
    public Servo flipperServo = null;


    // Constructor
    public FlipperSubsystem() {

    }

    // Initialize hardware variables
    public void init(HardwareMap hardwareMap, Telemetry telemetry) {
        super.init(hardwareMap, telemetry);
        flipperServo = hardwareMap.get(Servo.class,"flipperServo");
        flipperServo.setDirection(Servo.Direction.FORWARD);
    }

    // Default command
    public void defaultCommand() {
        if (gamepad2.y) {
            flipperServo.setPosition(FLIPPER_OPEN);
        } else {
            flipperServo.setPosition(FLIPPER_CLOSED);
        }
    }
}