package org.firstinspires.ftc.teamcode.hardware.subsystembase.main;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.subsystembase.base.BaseSubsystem;

@Config
public class TelemetrySubsystem extends BaseSubsystem {
    // Values
    AllianceMarkerStickSubsystem stick;
    ArmSubsystem arm;
    BoxSubsystem box;
    CarouselSubsystem carousel;
    DrivetrainSubsystem drivetrain;
    IntakeSubsystem intake;
    BreakModeSubsystem breakMode;
    public int cycles = 0;

    // Create hardware variables

    // Constructor
    public TelemetrySubsystem(Gamepad gamepad1, Gamepad gamepad2, HardwareMap hardwareMap, Telemetry telemetry, AllianceMarkerStickSubsystem stick, ArmSubsystem arm, BoxSubsystem box, CarouselSubsystem carousel, DrivetrainSubsystem drivetrain, IntakeSubsystem intake, BreakModeSubsystem breakMode) {
        super(gamepad1, gamepad2, hardwareMap, telemetry);
        this.stick = stick;
        this.arm = arm;
        this.box = box;
        this.carousel = carousel;
        this.drivetrain = drivetrain;
        this.intake = intake;
        this.breakMode = breakMode;
    }

    // Initialization message
    public void initMessage() {
        telemetry.addData("GL", "You better win!");
    }

    // Default command
    public void defaultCommand() {
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        cycles++;
        telemetry.addData("Frequency", (int) (cycles / runtime.seconds()) + "hz");
        telemetry.addData("Alliance Marker Servo Position", stick.allianceMarkerServoTargetPos);
        telemetry.addData("Break Mode status", breakMode.breakModeStatus);
    }
    public void verboseCommand() {
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        cycles++;
        telemetry.addData("Frequency", (int) (cycles / runtime.seconds()) + "hz");
        telemetry.addData("Alliance Marker Stick Servo Position", stick.allianceMarkerServoTargetPos);
        telemetry.addData("Arm Position", arm.getArmPosition());
        telemetry.addData("Intake Speed", intake.intakeMotor.getVelocity());
        telemetry.addData("Break Mode status", breakMode.breakModeStatus);
    }
}