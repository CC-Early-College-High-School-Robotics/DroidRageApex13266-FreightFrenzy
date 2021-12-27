package org.firstinspires.ftc.teamcode.teleop.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

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

@TeleOp(name="Show Tuning Results", group="test")
public class ShowTuningResults extends OpMode {

    Field[][] endTuning = {
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
            TurretSubsystem .class.getDeclaredFields(),
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

    List<Double> endTuningDoubles = new ArrayList<>();
    List<String> endTuningStrings = new ArrayList<>();
    boolean dataExists = false;
    boolean dataExistsInClass = false;
    Field tempField;


    @Override
    public void init() {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        TuningStart.initializeTuning();

        int doubleIndexNumber=0;
        int stringIndexNumber=0;

        // consider enhanced for loop
        for (Field[] fields : endTuning) {

            // go through every field in class
            for (Field field : fields) {
                try {
                    endTuningDoubles.add(field.getDouble(field));

                    if (!endTuningDoubles.get(doubleIndexNumber).equals(TuningStart.initialTuningDoubles.get(doubleIndexNumber))) {
                        telemetry.addData(field.getName(), endTuningDoubles.get(doubleIndexNumber));
                        dataExists = true;
                        dataExistsInClass = true;
                    }
                    doubleIndexNumber++;
                } catch (Exception ignored) {

                }
                try {
                    endTuningStrings.add((String) field.get(field));

                    // check if
                    if (!endTuningStrings.get(stringIndexNumber).equals(TuningStart.initialTuningStrings.get(stringIndexNumber))) {
                        telemetry.addData(field.getName(), endTuningStrings.get(stringIndexNumber));
                        dataExists = true;
                        dataExistsInClass = true;
                    }
                    stringIndexNumber++;
                } catch (Exception ignored) {

                }
                try {
                    endTuningDoubles.add(((PIDCoefficients) Objects.requireNonNull(field.get(field))).kP);
                    endTuningDoubles.add(((PIDCoefficients) Objects.requireNonNull(field.get(field))).kI);
                    endTuningDoubles.add(((PIDCoefficients) Objects.requireNonNull(field.get(field))).kD);

                    //check if the kp is not equal
                    if (!endTuningDoubles.get(doubleIndexNumber).equals(TuningStart.initialTuningDoubles.get(doubleIndexNumber))) {
                        telemetry.addData(field.getName() + " kP", endTuningDoubles.get(doubleIndexNumber));
                        dataExists = true;
                        dataExistsInClass = true;
                    }
                    doubleIndexNumber++;

                    // check if the ki is not equal
                    if (!endTuningDoubles.get(doubleIndexNumber).equals(TuningStart.initialTuningDoubles.get(doubleIndexNumber))) {
                        telemetry.addData(field.getName() + " kI", endTuningDoubles.get(doubleIndexNumber));
                        dataExists = true;
                    }
                    doubleIndexNumber++;

                    // check if kd is not equal
                    if (!endTuningDoubles.get(doubleIndexNumber).equals(TuningStart.initialTuningDoubles.get(doubleIndexNumber))) {
                        telemetry.addData(field.getName() + " kD", endTuningDoubles.get(doubleIndexNumber));
                        dataExists = true;
                        dataExistsInClass = true;
                    }
                    doubleIndexNumber++;
                } catch (Exception ignored) {

                }
                try {
                    endTuningDoubles.add(((PIDFCoefficients) Objects.requireNonNull(field.get(field))).p);
                    endTuningDoubles.add(((PIDFCoefficients) Objects.requireNonNull(field.get(field))).i);
                    endTuningDoubles.add(((PIDFCoefficients) Objects.requireNonNull(field.get(field))).d);
                    endTuningDoubles.add(((PIDFCoefficients) Objects.requireNonNull(field.get(field))).f);

                    //check if the p is not equal
                    if (!endTuningDoubles.get(doubleIndexNumber).equals(TuningStart.initialTuningDoubles.get(doubleIndexNumber))) {
                        telemetry.addData(field.getName() + " p", endTuningDoubles.get(doubleIndexNumber));
                        dataExists = true;
                        dataExistsInClass = true;
                    }
                    doubleIndexNumber++;

                    // check if the i is not equal
                    if (!endTuningDoubles.get(doubleIndexNumber).equals(TuningStart.initialTuningDoubles.get(doubleIndexNumber))) {
                        telemetry.addData(field.getName() + " i", endTuningDoubles.get(doubleIndexNumber));
                        dataExists = true;
                        dataExistsInClass = true;
                    }
                    doubleIndexNumber++;

                    // check if d is not equal
                    if (!endTuningDoubles.get(doubleIndexNumber).equals(TuningStart.initialTuningDoubles.get(doubleIndexNumber))) {
                        telemetry.addData(field.getName() + " d", endTuningDoubles.get(doubleIndexNumber));
                        dataExists = true;
                        dataExistsInClass = true;
                    }
                    doubleIndexNumber++;
                    // check if f is not equal
                    if (!endTuningDoubles.get(doubleIndexNumber).equals(TuningStart.initialTuningDoubles.get(doubleIndexNumber))) {
                        telemetry.addData(field.getName() + " f", endTuningDoubles.get(doubleIndexNumber));
                        dataExists = true;
                        dataExistsInClass = true;
                    }
                    doubleIndexNumber++;
                } catch (Exception ignored) {

                }
                tempField = field;
            }
            if (dataExistsInClass) {
                telemetry.addLine(tempField.getDeclaringClass().getSimpleName());
                telemetry.addLine();
            }
            dataExistsInClass = false;
        }
        if (!dataExists) {
            telemetry.addLine("There is no tuning data that has been changed and recorded :(");
        }
    }

    @Override
    public void loop() {
    }
}