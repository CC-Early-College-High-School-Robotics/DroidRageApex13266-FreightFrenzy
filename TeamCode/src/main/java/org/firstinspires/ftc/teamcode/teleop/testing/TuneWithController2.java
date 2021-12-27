
package org.firstinspires.ftc.teamcode.teleop.testing;

import static java.lang.Math.abs;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.auto.pipeline.一BlueCarouselDuckDetection;
import org.firstinspires.ftc.teamcode.auto.pipeline.一BlueWarehouseDuckDetection;
import org.firstinspires.ftc.teamcode.auto.pipeline.一RedCarouselDuckDetection;
import org.firstinspires.ftc.teamcode.auto.pipeline.一RedWarehouseDuckDetection;
import org.firstinspires.ftc.teamcode.hardware.subsystembase.main.AllianceMarkerStickSubsystem;
import org.firstinspires.ftc.teamcode.hardware.subsystembase.main.ArmSubsystem;
import org.firstinspires.ftc.teamcode.hardware.subsystembase.main.BoxSubsystem;
import org.firstinspires.ftc.teamcode.hardware.subsystembase.main.BreakModeSubsystem;
import org.firstinspires.ftc.teamcode.hardware.subsystembase.main.CameraStandSubsystem;
import org.firstinspires.ftc.teamcode.hardware.subsystembase.main.CarouselSubsystem;
import org.firstinspires.ftc.teamcode.hardware.subsystembase.main.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.hardware.subsystembase.main.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.hardware.subsystembase.main.TelemetrySubsystem;
import org.firstinspires.ftc.teamcode.hardware.一AutoValues;
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

@Config
@TeleOp(name="Tune With Controller2", group="test")
@Disabled
public class TuneWithController2 extends LinearOpMode {
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

//    public static List<Double> initialTuningDoubles = new ArrayList<>();
//    public static List<String> initialTuningStrings = new ArrayList<>();
    public static String selected = " ";
    public static int indexNumber = 0;
    public static int selectedIndexNumber = 0;
    public static boolean gamepad1a = false;
    public static boolean gamepad1b = false;
    public static boolean gamepad1left = false;
    public static boolean gamepad1right = false;
    public static boolean gamepad1up = false;
    public static boolean gamepad1down = false;
    public static boolean recalculateScale = true;
    public static int selectedScale = 1;
    public static double selectedScaleValue = 1;
    char selectionCharacter = '█';
    public static double currentDouble = 0;
    public static double currentKP = 0;
    public static double currentKI = 0;
    public static double currentKD = 0;

    Field tempField;
    boolean dataExistsInClass = false;

    private ElapsedTime runtime = new ElapsedTime();


    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        telemetry.setMsTransmissionInterval(10);


        while (!isStopRequested()) {

            indexNumber = 0;
            // Initialize subsystems
            for (Field[] fields : initialTuning) {

                // go through every field in class
                for (Field field : fields) {
                    if (indexNumber == selectedIndexNumber) {
                        selected = " <";
                    } else {
                        selected = "";
                    }

                    try {
                        currentDouble = field.getDouble(field);
                        scaleValueCalculation(currentDouble, field);
                        telemetry.addData(field.getName(), fieldBlinker(currentDouble) + selected);


                        if (gamepad1a || gamepad1.a) {
                            field.setDouble(field, currentDouble + selectedScaleValue);
                            gamepad1a = false;
                        }
                        if (gamepad1b || gamepad1.b) {
                            field.setDouble(field, currentDouble - selectedScaleValue);
                            gamepad1b = false;
                        }

                        dataExistsInClass = true;
                        indexNumber++;
                    } catch (Exception ignored) {

                    }

                    try {
                        currentKP = ((PIDCoefficients) Objects.requireNonNull(field.get(field))).kP;
                        scaleValueCalculation(currentKP, field);
                        telemetry.addData(field.getName() + " kP", fieldBlinker(currentKP) + selected);

                        if (gamepad1a || gamepad1.a) {
                            field.set(field, new PIDCoefficients(currentKP + selectedScaleValue, currentKI, currentKD));
                            gamepad1a = false;
                        }
                        if (gamepad1b || gamepad1.b) {
                            field.set(field, new PIDCoefficients(currentKP - selectedScaleValue, currentKI, currentKD));
                            gamepad1b = false;
                        }

                        indexNumber++;

                        currentKI = ((PIDCoefficients) Objects.requireNonNull(field.get(field))).kI;
                        scaleValueCalculation(currentKI, field);
                        telemetry.addData(field.getName() + " kI", fieldBlinker(currentKI) + selected);

                        if (gamepad1a || gamepad1.a) {
                            field.set(field, new PIDCoefficients(currentKP, currentKI + selectedScaleValue, currentKD));
                            gamepad1a = false;
                        }
                        if (gamepad1b || gamepad1.b) {
                            field.set(field, new PIDCoefficients(currentKP, currentKI - selectedScaleValue, currentKD));
                            gamepad1b = false;
                        }

                        indexNumber++;

                        currentKD = ((PIDCoefficients) Objects.requireNonNull(field.get(field))).kP;
                        scaleValueCalculation(currentKD, field);

                        telemetry.addData(field.getName() + " kD", fieldBlinker(currentKD) + selected);

                        if (gamepad1a || gamepad1.a) {
                            field.set(field, new PIDCoefficients(currentKP, currentKI, currentKD + selectedScaleValue));
                            gamepad1a = false;
                        }
                        if (gamepad1b || gamepad1.b) {
                            field.set(field, new PIDCoefficients(currentKP, currentKI, currentKD - selectedScaleValue));
                            gamepad1b = false;
                        }

                        dataExistsInClass = true;
                        indexNumber++;
                        /*
                        if (indexNumber == selectedIndexNumber) {

                            if (recalculateScale) {
                                selectedScale = findScale(field.getDouble(field));
                                recalculateScale = false;
                            }
                            if (gamepad1left) {
                                selectedScale = selectedScale + 1;
                                gamepad1left = false;
                            }
                            if (gamepad1right) {
                                selectedScale = selectedScale - 1;
                                gamepad1right = false;
                            }
                            selectedScaleValue = Math.pow(10, selectedScale);

                            if (gamepad1a || gamepad1.a) {
                                field.setDouble(field, field.getDouble(field) + selectedScaleValue);
                                gamepad1a = false;
                            }
                            if (gamepad1b || gamepad1.b) {
                                field.setDouble(field, field.getDouble(field) - selectedScaleValue);
                                gamepad1b = false;
                            }

//                            telemetry.addLine(String.valueOf(runtime.milliseconds()));
//                            telemetry.addLine(String.valueOf(runtime.milliseconds() % 100));
                        }

                        if (indexNumber == selectedIndexNumber && (((int) runtime.milliseconds()) % 1500) > 1100) {
                            char[] original = String.valueOf(field.getDouble(field)).toCharArray();

                            List<Character> numberWithSelector = new ArrayList<>();

                            int nonNumbers = 0;
                            for (int i = 0; i < original.length; i++) {
                                if (original[i] == '-' || original[i] == '.') {
                                    nonNumbers++;
                                    numberWithSelector.add(original[i]);
                                } else if (i == (nonNumbers + ((findScale(field.getDouble(field))) - selectedScale))) {
                                    numberWithSelector.add(selectionCharacter);
                                } else {
                                    numberWithSelector.add(original[i]);
                                }
                            }
                            StringBuilder sb = new StringBuilder();


                            for (Character ch: numberWithSelector) {
                                sb.append(ch);
                            }

                            String output = sb.toString();

                            telemetry.addData(field.getName(), output + selected);
                        }
                        else {
                            telemetry.addData(field.getName() + " kP", ((PIDCoefficients) Objects.requireNonNull(field.get(field))).kP + selected);
                        }



                        dataExistsInClass = true;
                        indexNumber++;

                         */
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
            if (gamepad1down || gamepad1.dpad_down) {
                selectedIndexNumber++;
                recalculateScale = true;
                gamepad1down = false;
            }
            if (gamepad1up || gamepad1.dpad_up) {
                selectedIndexNumber--;
                recalculateScale = true;
                gamepad1up = false;
            }
            telemetry.update();
        }

    }

    public int findScale(Double input) {
        boolean found = false;
        int outputScale = 0;
        while (!found) {
            if (input == 0) {
                found = true;
            }
            else if ((abs(input) / Math.pow(10, outputScale)) < 1) {
                outputScale--;
            } else if ((abs(input) / Math.pow(10, outputScale)) >= 10){
                outputScale++;
            } else {
                found = true;
                // outputScale++; // this is here to make it possible to tell the difference between a scale of 0 and a scale of 1. it also increases all of the scales so instead of 10 having a scale of 1, it has a scale of 2
            }
        }




        return outputScale;
    }
    public void scaleValueCalculation(Double current, Field currentField) {
        try {
            if (indexNumber == selectedIndexNumber) {

                if (recalculateScale) {
                    selectedScale = findScale(current);
                    recalculateScale = false;
                }
                if (gamepad1left || gamepad1.dpad_left) {
                    selectedScale = selectedScale + 1;
                    gamepad1left = false;
                }
                if (gamepad1right || gamepad1.dpad_right) {
                    selectedScale = selectedScale - 1;
                    gamepad1right = false;
                }
                selectedScaleValue = Math.pow(10, selectedScale);
            }
        } catch (Exception ignored) {

        }
    }
    public String fieldBlinker(Double current) {
        if (indexNumber == selectedIndexNumber && (((int) runtime.milliseconds()) % 1500) > 1100) {
            char[] original = String.valueOf(current).toCharArray();

            List<Character> numberWithSelector = new ArrayList<>();

            int nonNumbers = 0;
            for (int i = 0; i < original.length; i++) {
                if (original[i] == '-' || original[i] == '.') {
                    nonNumbers++;
                    numberWithSelector.add(original[i]);
                } else if (i == (nonNumbers + ((findScale(current)) - selectedScale))) {
                    numberWithSelector.add(selectionCharacter);
                } else {
                    numberWithSelector.add(original[i]);
                }
            }
            StringBuilder output = new StringBuilder();


            for (Character ch : numberWithSelector) {
                output.append(ch);
            }
            return output.toString();
        } else {
            return current.toString();
        }
    }
}