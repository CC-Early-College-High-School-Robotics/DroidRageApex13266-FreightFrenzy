package org.firstinspires.ftc.teamcode.hardware.subsystembase.main;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.Devices;
import org.firstinspires.ftc.teamcode.hardware.subsystembase.base.BaseSubsystem;

public class TurretSubsystem extends BaseSubsystem {
    // Values
    public static double TURRET_SERVO_1_FRONT =.210;
    public static double TURRET_SERVO_2_FRONT = .210;

    public static double TURRET_SERVO_1_LEFT = 0;
    public static double TURRET_SERVO_2_LEFT = 0;

    public static double TURRET_SERVO_1_RIGHT = .4838;
    public static double TURRET_SERVO_2_RIGHT = .4838;

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
            turretServo1.setPosition(TURRET_SERVO_1_FRONT);
            turretServo2.setPosition(TURRET_SERVO_2_FRONT);
        }
        // right
        if (gamepad2.b) {
            turretServo1.setPosition(TURRET_SERVO_1_RIGHT);
            turretServo2.setPosition(TURRET_SERVO_2_RIGHT);
        }
        // left
        if (gamepad2.x) {
            turretServo1.setPosition(TURRET_SERVO_1_LEFT);
            turretServo2.setPosition(TURRET_SERVO_2_LEFT);
        }
        // intake/reset position
        if (gamepad2.a) {
            turretServo1.setPosition(TURRET_SERVO_1_FRONT);
            turretServo2.setPosition(TURRET_SERVO_2_FRONT);
        }
    }
}
