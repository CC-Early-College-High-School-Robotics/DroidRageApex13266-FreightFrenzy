package org.firstinspires.ftc.teamcode.teleop.testing;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.auto.pipeline.一BlueCarouselDuckDetection;
import org.firstinspires.ftc.teamcode.auto.pipeline.一BlueWarehouseDuckDetection;
import org.firstinspires.ftc.teamcode.auto.pipeline.一RedCarouselDuckDetection;
import org.firstinspires.ftc.teamcode.auto.pipeline.一RedWarehouseDuckDetection;
import org.firstinspires.ftc.teamcode.hardware.subsystembase.main.FlipperSubsystem;
import org.firstinspires.ftc.teamcode.hardware.subsystembase.main.TurretSubsystem;
import org.firstinspires.ftc.teamcode.hardware.一AutoValues;
import org.firstinspires.ftc.teamcode.hardware.subsystembase.main.AllianceMarkerStickSubsystem;
import org.firstinspires.ftc.teamcode.hardware.subsystembase.main.ArmSubsystem;
import org.firstinspires.ftc.teamcode.hardware.subsystembase.main.BoxSubsystem;
import org.firstinspires.ftc.teamcode.hardware.subsystembase.main.BreakModeSubsystem;
import org.firstinspires.ftc.teamcode.hardware.subsystembase.main.CameraStandSubsystem;
import org.firstinspires.ftc.teamcode.hardware.subsystembase.main.CarouselSubsystem;
import org.firstinspires.ftc.teamcode.hardware.subsystembase.main.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.hardware.subsystembase.main.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.hardware.subsystembase.main.TelemetrySubsystem;
import org.firstinspires.ftc.teamcode.roadrunner.drive.RoadrunnerDriveConstants;
import org.firstinspires.ftc.teamcode.roadrunner.drive.RoadrunnerTankDrive;
import org.firstinspires.ftc.teamcode.roadrunner.drive.opmode.ￚAutomaticFeedforwardTuner;
import org.firstinspires.ftc.teamcode.roadrunner.drive.opmode.ￚBackAndForth;
import org.firstinspires.ftc.teamcode.roadrunner.drive.opmode.ￚDriveVelocityPIDTuner;
import org.firstinspires.ftc.teamcode.roadrunner.drive.opmode.ￚFollowerPIDTuner;
import org.firstinspires.ftc.teamcode.roadrunner.drive.opmode.ￚLocalizationTest;
import org.firstinspires.ftc.teamcode.roadrunner.drive.opmode.ￚManualFeedforwardTuner;
import org.firstinspires.ftc.teamcode.roadrunner.drive.opmode.ￚMaxAngularVeloTuner;
import org.firstinspires.ftc.teamcode.roadrunner.drive.opmode.ￚMaxVelocityTuner;
import org.firstinspires.ftc.teamcode.roadrunner.drive.opmode.ￚSplineTest;
import org.firstinspires.ftc.teamcode.roadrunner.drive.opmode.ￚStrafeTest;
import org.firstinspires.ftc.teamcode.roadrunner.drive.opmode.ￚTrackWidthTuner;
import org.firstinspires.ftc.teamcode.roadrunner.drive.opmode.ￚTrackingWheelForwardOffsetTuner;
import org.firstinspires.ftc.teamcode.roadrunner.drive.opmode.ￚTrackingWheelLateralDistanceTuner;
import org.firstinspires.ftc.teamcode.roadrunner.drive.opmode.ￚTurnTest;

import java.lang.reflect.Field;
import java.util.ArrayList;
import java.util.List;
import java.util.Objects;

public class TuningStart {
    public static Field[][] initialTuning = {
            RoadrunnerTankDrive.class.getDeclaredFields(),
            RoadrunnerDriveConstants.class.getDeclaredFields(),

            AllianceMarkerStickSubsystem.class.getDeclaredFields(),
            ArmSubsystem.class.getDeclaredFields(),
            BoxSubsystem.class.getDeclaredFields(),
            BreakModeSubsystem.class.getDeclaredFields(),
            CameraStandSubsystem.class.getDeclaredFields(),
            CarouselSubsystem.class.getDeclaredFields(),
            DrivetrainSubsystem.class.getDeclaredFields(),
            IntakeSubsystem.class.getDeclaredFields(),
            TelemetrySubsystem.class.getDeclaredFields(),
            TurretSubsystem.class.getDeclaredFields(),
            FlipperSubsystem.class.getDeclaredFields(),

            一AutoValues.class.getDeclaredFields(),

            一BlueCarouselDuckDetection.class.getDeclaredFields(),
            一BlueWarehouseDuckDetection.class.getDeclaredFields(),
            一RedCarouselDuckDetection.class.getDeclaredFields(),
            一RedWarehouseDuckDetection.class.getDeclaredFields(),

            ￚServoPositionProgrammer.class.getDeclaredFields(),
            ￚColorSensorProgrammer.class.getDeclaredFields(),

            ￚAutomaticFeedforwardTuner.class.getDeclaredFields(),
            ￚBackAndForth.class.getDeclaredFields(),
            ￚDriveVelocityPIDTuner.class.getDeclaredFields(),
            ￚFollowerPIDTuner.class.getDeclaredFields(),
            ￚLocalizationTest.class.getDeclaredFields(),
            ￚManualFeedforwardTuner.class.getDeclaredFields(),
            ￚMaxAngularVeloTuner.class.getDeclaredFields(),
            ￚMaxVelocityTuner.class.getDeclaredFields(),
            ￚSplineTest.class.getDeclaredFields(),
            ￚStrafeTest.class.getDeclaredFields(),
            ￚTrackingWheelForwardOffsetTuner.class.getDeclaredFields(),
            ￚTrackingWheelLateralDistanceTuner.class.getDeclaredFields(),
            ￚTrackWidthTuner.class.getDeclaredFields(),
            ￚTurnTest.class.getDeclaredFields()
    };

    public static List<Double> initialTuningDoubles = new ArrayList<>();
    public static List<String> initialTuningStrings = new ArrayList<>();
    public static boolean ranOnce = false;

    public static void initializeTuning() {
        // Initialize subsystems
        Telemetry telemetry = null;
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
                //telemetry.addData("amount of fields", fields.length);
            }
            //telemetry.addData("amount of classes", initialTuning.length);

            ranOnce = true;
        }

//        telemetry.addData("values", initialTuningDoubles);

    }
}