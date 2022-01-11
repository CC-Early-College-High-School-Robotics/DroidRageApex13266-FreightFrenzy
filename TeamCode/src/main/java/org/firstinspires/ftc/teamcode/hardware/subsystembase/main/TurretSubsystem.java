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
    public static double TURRET_SERVOS_FRONT = 0.49;

    public static double TURRET_SERVOS_LEFT = 0.14;

    public static double TURRET_SERVOS_RIGHT = 0.80;

    public static double TURRET_SERVOS_SPEED = 0.003;

    public static double TURRET_RANGE = 270;

    double targetPos = TURRET_SERVOS_FRONT;
    boolean disableTurret = true;

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

        setTurretPosition();
    }

    // Default command
    public void defaultCommand(Gamepad gamepad1, Gamepad gamepad2) {
        // Reset
        if (gamepad2.a && !disableTurret) {
            targetPos = TURRET_SERVOS_FRONT;
        }
        // right
        if (gamepad2.b && !disableTurret) {
            targetPos = TURRET_SERVOS_RIGHT;
        }
        // left
        if (gamepad2.x && !disableTurret) {
            targetPos = TURRET_SERVOS_LEFT;
        }
//        if (gamepad2.left_stick_x > 0.1 && !disableTurret) {
//            targetPos += TURRET_SERVOS_SPEED;
//        }
//        if (gamepad2.left_stick_x < -0.1 && !disableTurret) {
//            targetPos -= TURRET_SERVOS_SPEED;
//        }
        if ((gamepad2.a || gamepad2.b || gamepad2.x/* || gamepad2.left_stick_x > 0.1 || gamepad2.left_stick_x < -0.1*/) && !disableTurret) {
            setTurretPosition();
        }


    }
    public void setTurretPosition() {
        turretServo1.setPosition(targetPos);
        turretServo2.setPosition(targetPos);
    }
}
