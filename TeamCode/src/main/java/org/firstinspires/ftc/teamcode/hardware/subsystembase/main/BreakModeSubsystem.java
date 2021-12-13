package org.firstinspires.ftc.teamcode.hardware.subsystembase.main;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.subsystembase.base.BaseSubsystem;

@Config
public class BreakModeSubsystem extends BaseSubsystem {
    ArmSubsystem arm;
    CarouselSubsystem carousel;
    DrivetrainSubsystem drivetrain;
    String breakModeStatus;

    // Constructor
    public BreakModeSubsystem(Gamepad gamepad1, Gamepad gamepad2, HardwareMap hardwareMap, Telemetry telemetry, ArmSubsystem arm, CarouselSubsystem carousel, DrivetrainSubsystem drivetrain) {
        super(gamepad1, gamepad2, hardwareMap, telemetry);
        this.arm = arm;
        this.carousel = carousel;
        this.drivetrain = drivetrain;
    }

    // Default command
    public void defaultCommand() {
        if (gamepad1.a) {
            arm.breakMode();
            carousel.breakMode();
            drivetrain.breakMode();
            breakModeStatus = "On";
        }
        if (gamepad1.b) {
            arm.floatMode();
            carousel.floatMode();
            drivetrain.breakMode();
            breakModeStatus = "Off";
        }
    }
}