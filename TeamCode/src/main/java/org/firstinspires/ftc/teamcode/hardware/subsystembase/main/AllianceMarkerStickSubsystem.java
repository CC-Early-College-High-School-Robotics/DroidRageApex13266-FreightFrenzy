package org.firstinspires.ftc.teamcode.hardware.subsystembase.main;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.subsystembase.base.BaseSubsystem;

@Config
public class AllianceMarkerStickSubsystem extends BaseSubsystem {
    // Values
    public static double ALLIANCE_MARKER_SERVO_SPEED = 0.001;
    public static double ALLIANCE_MARKER_RESET_POS = 0;
    public static double ALLIANCE_MARKER_STANDING_POS = 0.787;
    public static double ALLIANCE_MARKER_KNOCKED_OVER_POS = 0.890;
    public static double ALLIANCE_MARKER_APPROACHING_HUB_POS = 0.443;
    public static double ALLIANCE_MARKER_CAPPED_HUB_POS = 0.550;

    public boolean leftBumperButtonPressed = false;
    public boolean rightBumperButtonPressed = false;
    public boolean allianceMarkerServoReset = true;
    public double allianceMarkerServoTargetPos;

    // Create hardware variables
    public Servo allianceMarkerServo = null;

    // Constructor
    public AllianceMarkerStickSubsystem() {
    }

    // Initialize hardware variables
    public void init(HardwareMap hardwareMap, Telemetry telemetry) {
        super.init(hardwareMap, telemetry);
        allianceMarkerServo = hardwareMap.get(Servo.class, "allianceMarkerServo");
        allianceMarkerServo.setDirection(Servo.Direction.FORWARD);

        allianceMarkerServoTargetPos = ALLIANCE_MARKER_RESET_POS;
        allianceMarkerServo.setPosition(allianceMarkerServoTargetPos);
    }

    // Default command
    public void defaultCommand(Gamepad gamepad1, Gamepad gamepad2) {
        super.gamepadInit(gamepad1, gamepad2);
        // Allow Picking Alliance Marker again if cap is not being used
        if (!gamepad2.left_bumper && !gamepad2.right_bumper && !rightBumperButtonPressed) {
            allianceMarkerServoReset = true;
        }

        // Picking up Alliance Marker
        if (gamepad2.left_bumper && !leftBumperButtonPressed && allianceMarkerServoReset) {
            allianceMarkerServoTargetPos = ALLIANCE_MARKER_STANDING_POS; //
            leftBumperButtonPressed = true;
        }
        if (!gamepad2.left_bumper && leftBumperButtonPressed && allianceMarkerServoReset) {
            allianceMarkerServoTargetPos = ALLIANCE_MARKER_KNOCKED_OVER_POS;
            leftBumperButtonPressed = false;
        }

        // Capping at hub
        if (gamepad2.right_bumper && !rightBumperButtonPressed) {
            allianceMarkerServoTargetPos = ALLIANCE_MARKER_APPROACHING_HUB_POS;
            rightBumperButtonPressed = true;
            allianceMarkerServoReset = false;
        }
        if (!gamepad2.right_bumper && rightBumperButtonPressed) {
            allianceMarkerServoTargetPos = ALLIANCE_MARKER_CAPPED_HUB_POS;
            rightBumperButtonPressed = false;
            allianceMarkerServoReset = false;
        }



        if (gamepad2.b && !gamepad2.left_bumper && !gamepad2.right_bumper) {
            allianceMarkerServoTargetPos = ALLIANCE_MARKER_RESET_POS;
            allianceMarkerServoReset = true;
        }

        if (-gamepad2.left_stick_y > ControllerSubsystem.TRIGGER_THRESHOLD) {
            allianceMarkerServoTargetPos -= ALLIANCE_MARKER_SERVO_SPEED;
        }
        if (-gamepad2.left_stick_y < -ControllerSubsystem.TRIGGER_THRESHOLD) {
            allianceMarkerServoTargetPos += ALLIANCE_MARKER_SERVO_SPEED;
        }

        allianceMarkerServo.setPosition(allianceMarkerServoTargetPos);
    }
}