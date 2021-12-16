package org.firstinspires.ftc.teamcode.teleop.main;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.subsystembase.main.AllianceMarkerStickSubsystem;
import org.firstinspires.ftc.teamcode.hardware.subsystembase.main.ArmSubsystem;
import org.firstinspires.ftc.teamcode.hardware.subsystembase.main.BoxSubsystem;
import org.firstinspires.ftc.teamcode.hardware.subsystembase.main.BreakModeSubsystem;
import org.firstinspires.ftc.teamcode.hardware.subsystembase.main.CarouselSubsystem;
import org.firstinspires.ftc.teamcode.hardware.subsystembase.main.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.hardware.subsystembase.main.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.hardware.subsystembase.main.TelemetrySubsystem;
import org.firstinspires.ftc.teamcode.teleop.testing.TuningStart;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="TeleOP Main")
public class TeleOpMain extends OpMode {
    // Declare subsystems
    AllianceMarkerStickSubsystem stick          = new AllianceMarkerStickSubsystem          ();
    ArmSubsystem arm                            = new ArmSubsystem                          ();
    BoxSubsystem box                            = new BoxSubsystem                          ();
    CarouselSubsystem carousel                  = new CarouselSubsystem                     ();
    DrivetrainSubsystem drivetrain              = new DrivetrainSubsystem                   ();
    IntakeSubsystem intake                      = new IntakeSubsystem                       ();
    BreakModeSubsystem breakMode                = new BreakModeSubsystem                    ();
    TelemetrySubsystem telemetrySubsystem       = new TelemetrySubsystem                    ();

    @Override
    public void init() {
        // Initialize subsystems
        stick.init(hardwareMap, telemetry);
        arm.init(hardwareMap, telemetry);
        box.init(hardwareMap, telemetry);
        carousel.init(hardwareMap, telemetry);
        drivetrain.init(hardwareMap, telemetry);
        intake.init(hardwareMap, telemetry);

        telemetrySubsystem.init(telemetry, stick, arm, box, carousel, drivetrain, intake, breakMode);
        breakMode.init(gamepad1, gamepad2, arm, carousel, drivetrain);

        TuningStart.initializeTuning();
        telemetrySubsystem.initMessage();
    }

    @Override
    public void loop() {
        stick.defaultCommand(gamepad1, gamepad2);
        arm.findArmPosition(gamepad1, gamepad2);
        arm.setArmPosition(gamepad1, gamepad2);
        box.defaultCommand(gamepad1, gamepad2);
        carousel.defaultCommand(gamepad1, gamepad2);
        drivetrain.defaultCommand(gamepad1, gamepad2);
        intake.defaultCommand(gamepad1, gamepad2);
        breakMode.defaultCommand();
        telemetrySubsystem.defaultCommand();
    }
}