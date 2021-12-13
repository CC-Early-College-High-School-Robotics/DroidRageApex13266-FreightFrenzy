package org.firstinspires.ftc.teamcode.hardware.subsystembase.main;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.Devices;
import org.firstinspires.ftc.teamcode.hardware.subsystembase.base.BaseSubsystem;

@Config
public class IntakeSubsystem extends BaseSubsystem {
    // Values
    public static double INTAKE_VELOCITY = 1000;

    // Create hardware variables
    public DcMotorEx intakeMotor = null;

    // Constructor
    public IntakeSubsystem(Gamepad gamepad1, Gamepad gamepad2, HardwareMap hardwareMap, Telemetry telemetry) {
        super(gamepad1, gamepad2, hardwareMap, telemetry);
    }

    // Initialize hardware variables
    public void init() {

        // Initialize hardware variables
        intakeMotor = hardwareMap.get(DcMotorEx.class,"intakeMotor");

        intakeMotor.setDirection(DcMotorEx.Direction.REVERSE);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        intakeMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }

    // Default command
    public void defaultCommand() {
        if (gamepad1.right_trigger >= TRIGGER_THRESHOLD) {
            intakeMotor.setVelocity(-INTAKE_VELOCITY);
        }
        else if (gamepad1.left_trigger >= TRIGGER_THRESHOLD) {
            intakeMotor.setVelocity(INTAKE_VELOCITY);
        }
        else if (gamepad1.right_trigger < TRIGGER_THRESHOLD && gamepad1.left_trigger < TRIGGER_THRESHOLD) {
            intakeMotor.setVelocity(0);
        }
    }
}