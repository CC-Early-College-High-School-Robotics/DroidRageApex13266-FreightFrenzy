package org.firstinspires.ftc.teamcode.hardware.subsystembase.main;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.subsystembase.base.BaseSubsystem;

@Config
public class TurretSubsystem extends BaseSubsystem {
    // Values
    public static double TURRET_SERVOS_FRONT = .23;

    public static double TURRET_SERVOS_LEFT = 0;

    public static double TURRET_SERVOS_RIGHT = .4838;

    public static double TURRET_SERVOS_SPEED = 0.2;

    double currentPos = 0.20;

    // Create hardware variables
    public Servo turretServo1 = null;
    public Servo turretServo2 = null;
    // Constructor
    public TurretSubsystem() {

    }

    // Initialize hardware variables
    public void init(HardwareMap hardwareMap, Telemetry telemetry) {
        super.init(hardwareMap, telemetry);
        turretServo1 = hardwareMap.get(Servo.class,"turretServo1");
        turretServo2 = hardwareMap.get(Servo.class,"turretServo2");
        turretServo1.setDirection(Servo.Direction.FORWARD);
        turretServo1.setDirection(Servo.Direction.FORWARD);
    }

    // Default command
    public void defaultCommand(Gamepad gamepad1, Gamepad gamepad2) {
        super.gamepadInit(gamepad1, gamepad2);
        // Reset
        if (gamepad2.a) {
            currentPos = TURRET_SERVOS_FRONT;
        }
        // right
        if (gamepad2.b) {
            currentPos = TURRET_SERVOS_RIGHT;
        }
        // left
        if (gamepad2.x) {
            currentPos = TURRET_SERVOS_LEFT;
        }
        if (gamepad2.left_stick_x > 0.1) {
            currentPos += TURRET_SERVOS_SPEED;
        }
        if (gamepad2.left_stick_x < -0.1) {
            currentPos -= TURRET_SERVOS_SPEED;
        }
        if (gamepad2.a || gamepad2.b || gamepad2.x || gamepad2.left_stick_x > 0.1 || gamepad2.left_stick_x < -0.1) {
            turretServo1.setPosition(currentPos);
            turretServo2.setPosition(currentPos);
        }


    }
}
