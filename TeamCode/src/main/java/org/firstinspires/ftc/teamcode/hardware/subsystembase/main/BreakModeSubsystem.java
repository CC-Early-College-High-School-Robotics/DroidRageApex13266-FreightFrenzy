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
    String breakModeStatus = "On";

    // Constructor
    public BreakModeSubsystem() {
    }

    // Default command
    public void init(Gamepad gamepad1, Gamepad gamepad2, ArmSubsystem arm, CarouselSubsystem carousel, DrivetrainSubsystem drivetrain) {
        super.gamepadInit(gamepad1, gamepad2);
        this.arm = arm;
        this.carousel = carousel;
        this.drivetrain = drivetrain;
    }
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
            drivetrain.floatMode();
            breakModeStatus = "Off";
        }
    }
}