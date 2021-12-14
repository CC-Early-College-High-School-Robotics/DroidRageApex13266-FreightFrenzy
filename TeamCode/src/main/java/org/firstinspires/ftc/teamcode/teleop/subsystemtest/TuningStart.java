package org.firstinspires.ftc.teamcode.teleop.subsystemtest;

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

@TeleOp(name="Tuning Start")
public class TuningStart extends OpMode {
    static Field[][] initialTuning = {
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
    static List<Double> initialTuningDoubles = new ArrayList<Double>();
    static List<String> initialTuningStrings = new ArrayList<String>();
//    static final List<PIDCoefficients> initialTuningPIDF = new ArrayList<PIDCoefficients>();
    static boolean ranOnce = false;

//    Field[] endTuning3 = ArmSubsystem.class.getDeclaredFields();
//    List<Double> endTuning3doubles = new ArrayList<Double>();
//    List<String> endTuning3strings = new ArrayList<String>();


    static PIDCoefficients tempPIDF = null;
    static PIDCoefficients tempPIDF2 = null;
    public static PIDCoefficients AXIAL_PID = new PIDCoefficients(9, 0, .5);

    @Override
    public void init() {
        // Initialize subsystems
        if (!ranOnce) {

            // consider enhanced for loop
            // go through every class
//            for (int e = 0; e < initialTuning.length; e++) {
//
//                // go through every field in class
//                for (int i = 0; i < initialTuning[e].length; i++) {
//                    // is field a double
//                    try {
//                        initialTuningDoubles.add(initialTuning[e][i].getDouble(initialTuning[e][i]));
//
//                    } catch (Exception noDouble) {
//                        // is field a string?
//                        try {
//                            initialTuningStrings.add((String) initialTuning[e][i].get(initialTuning[e][i]));
//                        } catch (Exception noString) {
//                            // is field a PIDCoefficients?
//                            try {
//                                initialTuningDoubles.add(((PIDCoefficients) initialTuning[e][i].get(initialTuning[e][i])).kP);
//                                initialTuningDoubles.add(((PIDCoefficients) initialTuning[e][i].get(initialTuning[e][i])).kI);
//                                initialTuningDoubles.add(((PIDCoefficients) initialTuning[e][i].get(initialTuning[e][i])).kD);
//                            } catch (Exception ignored) {
//
//                            }
//
//                        }
//                    }
//
//
//                }
//                telemetry.addData("amount of fields", initialTuning[e].length);
//            }
            for (int e = 0; e < initialTuning.length; e++) {

                // go through every field in class
                for (int i = 0; i < initialTuning[e].length; i++) {
                    // is field a double
                    try {
                        initialTuningDoubles.add(initialTuning[e][i].getDouble(initialTuning[e][i]));
                    } catch (Exception ignored) {

                    }
                    try {
                        initialTuningStrings.add((String) initialTuning[e][i].get(initialTuning[e][i]));
                    } catch (Exception ignored) {

                    }
                    try {
//                        tempPIDF = (PIDCoefficients) initialTuning[e][i].get(initialTuning[e][i]);
//                        tempPIDF2 = new PIDCoefficients(
//                                ((PIDCoefficients) initialTuning[e][i].get(initialTuning[e][i])).kP,
//                                ((PIDCoefficients) initialTuning[e][i].get(initialTuning[e][i])).kI,
//                                ((PIDCoefficients) initialTuning[e][i].get(initialTuning[e][i])).kD)
                                ;

                        initialTuningDoubles.add(((PIDCoefficients) initialTuning[e][i].get(initialTuning[e][i])).kP);
                        initialTuningDoubles.add(((PIDCoefficients) initialTuning[e][i].get(initialTuning[e][i])).kI);
                        initialTuningDoubles.add(((PIDCoefficients) initialTuning[e][i].get(initialTuning[e][i])).kD);

//                        tempPIDF = new PIDCoefficients(PIDCoefficients) initialTuning[e][i].get(initialTuning[e][i]);
//                        initialTuningDoubles.add(tempPIDF.kP);
//                        initialTuningDoubles.add(tempPIDF.kI);
//                        initialTuningDoubles.add(tempPIDF.kD);
//                        tempPIDF.kP = 0;
//                        tempPIDF.kI = 0;
//                        tempPIDF.kD = 0;

//                        initialTuningPIDF.add(((PIDCoefficients) initialTuning[e][i].get(initialTuning[e][i])));
//
//                        initialTuningDoubles.add(initialTuningPIDF.get(0).kP);
//                        initialTuningDoubles.add(initialTuningPIDF.get(0).kI);
//                        initialTuningDoubles.add(initialTuningPIDF.get(0).kD);
//
//                        initialTuningPIDF.remove(0);

//                        initialTuningDoubles.add(((PIDCoefficients) initialTuning[e][i].get(initialTuning[e][i])).kP);
//                        initialTuningDoubles.add(((PIDCoefficients) initialTuning[e][i].get(initialTuning[e][i])).kI);
//                        initialTuningDoubles.add(((PIDCoefficients) initialTuning[e][i].get(initialTuning[e][i])).kD);
                    } catch (Exception ignored) {

                    }
                    try {
                        initialTuningDoubles.add(((PIDFCoefficients) initialTuning[e][i].get(initialTuning[e][i])).p);
                        initialTuningDoubles.add(((PIDFCoefficients) initialTuning[e][i].get(initialTuning[e][i])).i);
                        initialTuningDoubles.add(((PIDFCoefficients) initialTuning[e][i].get(initialTuning[e][i])).d);
                        initialTuningDoubles.add(((PIDFCoefficients) initialTuning[e][i].get(initialTuning[e][i])).f);
                    }
                    catch (Exception ignored) {

                    }
                }
                telemetry.addData("amount of fields", initialTuning[e].length);
            }
            telemetry.addData("amount of classes", initialTuning.length);

            ranOnce = true;
        }

        telemetry.addData("values", initialTuningDoubles);
//        telemetry.addData("temppidf kp", tempPIDF2.kP);


//        telemetry.addData("hi", AXIAL_PID.component1());

    }

    @Override
    public void loop() {
    }
}