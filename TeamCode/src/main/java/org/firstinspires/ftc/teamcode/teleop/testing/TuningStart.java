package org.firstinspires.ftc.teamcode.teleop.testing;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.AutoValues;
import org.firstinspires.ftc.teamcode.hardware.subsystembase.main.AllianceMarkerStickSubsystem;
import org.firstinspires.ftc.teamcode.hardware.subsystembase.main.ArmSubsystem;
import org.firstinspires.ftc.teamcode.hardware.subsystembase.main.BoxSubsystem;
import org.firstinspires.ftc.teamcode.hardware.subsystembase.main.BreakModeSubsystem;
import org.firstinspires.ftc.teamcode.hardware.subsystembase.main.CameraStandSubsystem;
import org.firstinspires.ftc.teamcode.hardware.subsystembase.main.CarouselSubsystem;
import org.firstinspires.ftc.teamcode.hardware.subsystembase.main.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.hardware.subsystembase.main.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.hardware.subsystembase.main.TelemetrySubsystem;
import org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.roadrunner.drive.TankDriveRoadRunner;

import java.lang.reflect.Field;
import java.util.ArrayList;
import java.util.List;
import java.util.Objects;

@TeleOp(name="Tuning Start")
public class TuningStart extends OpMode {
    public static Field[][] initialTuning = {
            TankDriveRoadRunner.class.getDeclaredFields(),
            DriveConstants.class.getDeclaredFields(),
            AllianceMarkerStickSubsystem.class.getDeclaredFields(),
            ArmSubsystem.class.getDeclaredFields(),
            BoxSubsystem.class.getDeclaredFields(),
            BreakModeSubsystem.class.getDeclaredFields(),
            CameraStandSubsystem.class.getDeclaredFields(),
            CarouselSubsystem.class.getDeclaredFields(),
            DrivetrainSubsystem.class.getDeclaredFields(),
            IntakeSubsystem.class.getDeclaredFields(),
            TelemetrySubsystem.class.getDeclaredFields(),
            AutoValues.class.getDeclaredFields()
    };

    public static List<Double> initialTuningDoubles = new ArrayList<>();
    public static List<String> initialTuningStrings = new ArrayList<>();
    static boolean ranOnce = false;

    @Override
    public void init() {
        // Initialize subsystems
        if (!ranOnce) {
            for (Field[] fields : initialTuning) {

                // go through every field in class
                for (Field field : fields) {
                    // is field a double
                    try {
                        initialTuningDoubles.add(field.getDouble(field));
                    } catch (Exception ignored) {
                    }
                    try {
                        initialTuningStrings.add((String) field.get(field));
                    } catch (Exception ignored) {
                    }
                    try {
                        initialTuningDoubles.add(((PIDCoefficients) Objects.requireNonNull(field.get(field))).kP);
                        initialTuningDoubles.add(((PIDCoefficients) Objects.requireNonNull(field.get(field))).kI);
                        initialTuningDoubles.add(((PIDCoefficients) Objects.requireNonNull(field.get(field))).kD);
                    } catch (Exception ignored) {
                    }
                    try {
                        initialTuningDoubles.add(((PIDFCoefficients) Objects.requireNonNull(field.get(field))).p);
                        initialTuningDoubles.add(((PIDFCoefficients) Objects.requireNonNull(field.get(field))).i);
                        initialTuningDoubles.add(((PIDFCoefficients) Objects.requireNonNull(field.get(field))).d);
                        initialTuningDoubles.add(((PIDFCoefficients) Objects.requireNonNull(field.get(field))).f);
                    } catch (Exception ignored) {
                    }
                }
                telemetry.addData("amount of fields", fields.length);
            }
            telemetry.addData("amount of classes", initialTuning.length);

            ranOnce = true;
        }

        telemetry.addData("values", initialTuningDoubles);

    }

    @Override
    public void loop() {
    }
}