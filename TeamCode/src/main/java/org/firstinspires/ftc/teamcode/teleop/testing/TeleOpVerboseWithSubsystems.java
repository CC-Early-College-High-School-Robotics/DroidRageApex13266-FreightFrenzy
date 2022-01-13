package org.firstinspires.ftc.teamcode.teleop.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.subsystembase.main.AllianceMarkerStickSubsystem;
import org.firstinspires.ftc.teamcode.hardware.subsystembase.main.ArmSubsystem;
import org.firstinspires.ftc.teamcode.hardware.subsystembase.main.BoxSubsystem;
import org.firstinspires.ftc.teamcode.hardware.subsystembase.main.BreakModeSubsystem;
import org.firstinspires.ftc.teamcode.hardware.subsystembase.main.CarouselSubsystem;
import org.firstinspires.ftc.teamcode.hardware.subsystembase.main.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.hardware.subsystembase.main.FlipperSubsystem;
import org.firstinspires.ftc.teamcode.hardware.subsystembase.main.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.hardware.subsystembase.main.TelemetrySubsystem;
import org.firstinspires.ftc.teamcode.hardware.subsystembase.main.TurretSubsystem;

@TeleOp(name="TeleOP Verbose")
public class TeleOpVerboseWithSubsystems extends LinearOpMode {
    // Declare subsystems
    ArmSubsystem arm                            = new ArmSubsystem                          ();
    BoxSubsystem box                            = new BoxSubsystem                          ();
    CarouselSubsystem carousel                  = new CarouselSubsystem                     ();
    DrivetrainSubsystem drivetrain              = new DrivetrainSubsystem                   ();
    IntakeSubsystem intake                      = new IntakeSubsystem                       ();
    BreakModeSubsystem breakMode                = new BreakModeSubsystem                    ();
    TelemetrySubsystem telemetrySubsystem       = new TelemetrySubsystem                    ();
    TurretSubsystem turret                      = new TurretSubsystem                       ();
    FlipperSubsystem flipper                    = new FlipperSubsystem                      ();

    @Override
    public void runOpMode() {
        // Initialize subsystems
//        stick.init(hardwareMap, telemetry);
//        arm.init(hardwareMap, telemetry, box, turret, flipper);
        box.init(hardwareMap, telemetry);
        carousel.init(hardwareMap, telemetry);
        drivetrain.init(hardwareMap, telemetry);
        intake.init(hardwareMap, telemetry);
        turret.init(hardwareMap, telemetry);
        flipper.init(hardwareMap, telemetry);

        telemetrySubsystem.init(telemetry, arm, box, carousel, drivetrain, intake, breakMode);
        breakMode.init(gamepad1, gamepad2, arm, carousel, drivetrain);

        TuningStart.initializeTuning();
        telemetrySubsystem.initMessage();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        telemetrySubsystem.resetRuntime();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

//            stick.defaultCommand(gamepad1, gamepad2);
            arm.findArmPosition(gamepad1, gamepad2);
            arm.loopCommand();

//            box.defaultCommand(gamepad1, gamepad2);
            carousel.defaultCommand(gamepad1, gamepad2);
            drivetrain.defaultCommand(gamepad1, gamepad2);
            intake.defaultCommand(gamepad1, gamepad2);
            breakMode.defaultCommand();
            turret.defaultCommand(gamepad1, gamepad2);
            flipper.defaultCommand(gamepad1, gamepad2);
            telemetrySubsystem.verboseCommand();
        }
    }
}