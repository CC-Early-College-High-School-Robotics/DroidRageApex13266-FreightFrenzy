package org.firstinspires.ftc.teamcode.hardware.subsystembase.main;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.subsystembase.base.BaseSubsystem;

@Config
public class CameraStandSubsystem extends BaseSubsystem {
    // Values
    public static double CAMERA_BLUE_CAROUSEL_POS = 0.29;
    public static double CAMERA_BLUE_WAREHOUSE_POS = 0.24;
    public static double CAMERA_RED_CAROUSEL_POS = 0.55;
    public static double CAMERA_RED_WAREHOUSE_POS = 0.60;

    public static double WAIT_CAMERA_LENGTH = 4000;
    public long CAMERA_WAIT_TIME = (long) WAIT_CAMERA_LENGTH;

    // Create hardware variables
    public Servo cameraServo = null;

    // Constructor
    public CameraStandSubsystem(Gamepad gamepad1, Gamepad gamepad2, HardwareMap hardwareMap, Telemetry telemetry) {
        super(gamepad1, gamepad2);
    }

    // Initialize hardware variables
    public void init() {
        super.init(hardwareMap, telemetry);
        cameraServo = hardwareMap.get(Servo.class,"cameraServo");
        cameraServo.setDirection(Servo.Direction.FORWARD);
    }

    // Default command
    public void setPosition(double pos) {
        cameraServo.setPosition(pos);
    }
}