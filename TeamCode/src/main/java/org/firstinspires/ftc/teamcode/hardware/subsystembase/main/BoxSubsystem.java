package org.firstinspires.ftc.teamcode.hardware.subsystembase.main;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.subsystembase.base.BaseSubsystem;

@Config
public class BoxSubsystem extends BaseSubsystem {
    // Values
    public static double BOX_HIGH = 0.892;
    public static double BOX_SHARED = 1.0;
//    public static double BOX_AUTO_APPROACH_HUB = 0.500;
    public static double BOX_DOWN_SLIGHTLY_FORWARD = 0.1;
    public static double BOX_DOWN = 0.05;
//    public static double BOX_INTAKE_FORWARD = 0.02;
//    public static double BOX_DROP = 0.180;
//    public static double BOX_INTERMEDIATE = 0.3;


    public static double BOX_UP_WAIT = 0.2;
    public static double BOX_INTAKE_WAIT = 0.45;
    public static double BOX_INTAKE_SECOND_WAIT = 0.2;

    // Create hardware variables
    public Servo boxServo = null;
    double targetTime = .35;
    boolean setUpPos = false;
    boolean setIntakePos = false;
    boolean setIntakeSecondPos = false;

    // Constructor
    public BoxSubsystem() {

    }

    // Initialize hardware variables
    public void init(HardwareMap hardwareMap, Telemetry telemetry) {
        super.init(hardwareMap, telemetry);
        boxServo = hardwareMap.get(Servo.class,"boxServo");
        boxServo.setDirection(Servo.Direction.REVERSE);

        boxServo.setPosition(BOX_DOWN);
    }

    // Default command
    public void defaultCommand(Gamepad gamepad1, Gamepad gamepad2) {
        super.gamepadInit(gamepad1, gamepad2);
        // High
//        if (gamepad2.dpad_up) {
//            targetTime = runtime.seconds() + BOX_UP_WAIT;
//            setUpPos = true;
//        }
//        if (setUpPos && runtime.seconds() >= targetTime) {
//            boxServo.setPosition(BOX_FORWARD);
//            setUpPos = false;
//        }
//
//        // Mid
//        if (gamepad2.dpad_right) {
//            boxServo.setPosition(BOX_FORWARD);
//        }
//        // Shared hub
//        if (gamepad2.dpad_down) {
//            boxServo.setPosition(BOX_FORWARD);
//
//        }
//        // intake/reset position
//        if (gamepad2.dpad_left) {
//            boxServo.setPosition(BOX_INTERMEDIATE);
//
//            setIntakePos = true;
//            targetTime = runtime.seconds() + BOX_INTAKE_WAIT;
//        }
//        if (setIntakePos && runtime.seconds() >= targetTime) {
//            boxServo.setPosition(BOX_INTAKE_FORWARD);
//            setIntakePos = false;
//            setIntakeSecondPos = true;
//            targetTime = runtime.seconds() + BOX_INTAKE_SECOND_WAIT;
//        }
//        if (setIntakeSecondPos && runtime.seconds() >= targetTime) {
//            boxServo.setPosition(BOX_DOWN);
//            setIntakeSecondPos = false;
//        }




//        if (gamepad2.a) {
//            boxServo.setPosition(Devices.BOX_INTAKE);
//        }
//        if (gamepad2.x) {
//            boxServo.setPosition(Devices.BOX_DROP);
//        }
//        if (gamepad2.y) {
//            boxServo.setPosition(Devices.BOX_UP);
//        }
    }
}