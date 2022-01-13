package org.firstinspires.ftc.teamcode.hardware.subsystembase.main;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
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
    DistanceSensorSubsystem distanceSensor;
    public int cycles = 0;

    // Create hardware variables

    // Constructor
    public TelemetrySubsystem() {
    }

    // Initialization
    public void init(Telemetry telemetry, AllianceMarkerStickSubsystem stick, ArmSubsystem arm, BoxSubsystem box, CarouselSubsystem carousel, DrivetrainSubsystem drivetrain, IntakeSubsystem intake, BreakModeSubsystem breakMode) {
        this.telemetry = telemetry;
        this.stick = stick;
        this.arm = arm;
        this.box = box;
        this.carousel = carousel;
        this.drivetrain = drivetrain;
        this.intake = intake;
        this.breakMode = breakMode;
    }

    // Initialization
    public void init(Telemetry telemetry, ArmSubsystem arm, BoxSubsystem box, CarouselSubsystem carousel, DrivetrainSubsystem drivetrain, IntakeSubsystem intake, DistanceSensorSubsystem distanceSensor) {
        this.telemetry = telemetry;
        this.stick = stick;
        this.arm = arm;
        this.box = box;
        this.carousel = carousel;
        this.drivetrain = drivetrain;
        this.intake = intake;
        this.distanceSensor = distanceSensor;
    }

    // Initialization
    public void init(Telemetry telemetry, ArmSubsystem arm, BoxSubsystem box, CarouselSubsystem carousel, DrivetrainSubsystem drivetrain, IntakeSubsystem intake, BreakModeSubsystem breakMode) {
        this.telemetry = telemetry;
        this.arm = arm;
        this.box = box;
        this.carousel = carousel;
        this.drivetrain = drivetrain;
        this.intake = intake;
        this.breakMode = breakMode;
    }

    public void initMessage() {
        telemetry.addData("GL", "You better win!");
        telemetry.update();
    }

    // Default command
    public void defaultCommand() {
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        cycles++;
        telemetry.addData("Frequency", (int) (cycles / runtime.seconds()) + "hz");
//        telemetry.addData("Alliance Marker Servo Position", stick.allianceMarkerServoTargetPos);
//        telemetry.addData("Break Mode status", breakMode.breakModeStatus);
        telemetry.addData("distance sensor distance in MM", distanceSensor.distanceSensor.getDistance(DistanceUnit.MM));
        telemetry.update();
    }
    public void verboseCommand() {
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        cycles++;
        telemetry.addData("Frequency", (int) (cycles / runtime.seconds()) + "hz");
//        telemetry.addData("Alliance Marker Stick Servo Position", stick.allianceMarkerServoTargetPos);
        telemetry.addData("Arm Position", arm.getArmPosition());
        telemetry.addData("Arm PIDF Coeffiecients", arm.armMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION));
        telemetry.addData("Intake Speed", intake.intakeMotor.getVelocity());
        telemetry.addData("Break Mode status", breakMode.breakModeStatus);
        telemetry.update();
    }

    public void cyclesCommand() {
        cycles++;
        telemetry.addData("Frequency", (int) (cycles / runtime.seconds()) + "hz");
        telemetry.addData("Cycles", cycles);
        telemetry.update();
    }
    public void resetRuntime() {
        runtime.reset();
    }
}