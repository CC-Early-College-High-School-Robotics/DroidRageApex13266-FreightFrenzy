package org.firstinspires.ftc.teamcode.teleop.subsystemtest;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

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

@TeleOp(name="Tuning Stop")
public class TuningStop extends OpMode {
//    static Field[] initialTuning3 = ArmSubsystem.class.getDeclaredFields();
//    List<Double> initialTuning3doubles = new ArrayList<Double>();
//    List<String> initialTuning3strings = new ArrayList<String>();

    Field[][] endTuning = {
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

    List<Double> endTuningDoubles = new ArrayList<Double>();
    List<String> endTuningStrings = new ArrayList<String>();
    List<PIDCoefficients> endTuningPIDF = new ArrayList<PIDCoefficients>();
    boolean dataExists = false;

    @Override
    public void init() {
        int doubleIndexNumber=0;
        int stringIndexNumber=0;

        // consider enhanced for loop
        for (int e = 0; e < endTuning.length; e++) {

            // go through every field in class
            for (int i = 0; i < endTuning[e].length; i++) {
                try {
                    endTuningDoubles.add(endTuning[e][i].getDouble(endTuning[e][i]));

                    if (!endTuningDoubles.get(doubleIndexNumber).equals(TuningStart.initialTuningDoubles.get(doubleIndexNumber))) {
                        telemetry.addData(endTuning[e][i].getName(), endTuningDoubles.get(doubleIndexNumber));
                        dataExists = true;
                    }
                    doubleIndexNumber++;
                } catch (Exception ignored) {

                }
                try {
                    endTuningStrings.add((String) endTuning[e][i].get(endTuning[e][i]));

                    // check if
                    if (!endTuningStrings.get(stringIndexNumber).equals(TuningStart.initialTuningStrings.get(stringIndexNumber))) {
                        telemetry.addData(endTuning[e][i].getName(), endTuningStrings.get(stringIndexNumber));
                        dataExists = true;
                    }
                    stringIndexNumber++;
                } catch (Exception ignored) {

                }
                try {
                    endTuningDoubles.add(((PIDCoefficients) endTuning[e][i].get(endTuning[e][i])).kP);
                    endTuningDoubles.add(((PIDCoefficients) endTuning[e][i].get(endTuning[e][i])).kI);
                    endTuningDoubles.add(((PIDCoefficients) endTuning[e][i].get(endTuning[e][i])).kD);

//                    endTuningPIDF.add(((PIDCoefficients) endTuning[e][i].get(endTuning[e][i])));
//
//                    endTuningDoubles.add(endTuningPIDF.get(0).kP);
//                    endTuningDoubles.add(endTuningPIDF.get(1).kI);
//                    endTuningDoubles.add(endTuningPIDF.get(2).kD);
//
//                    endTuningPIDF.remove(0);
//                    endTuningPIDF.remove(1);
//                    endTuningPIDF.remove(2);
//                    endTuningDoubles.add(((PIDCoefficients) endTuning[e][i].get(endTuning[e][i])).kP);
//                    endTuningDoubles.add(((PIDCoefficients) endTuning[e][i].get(endTuning[e][i])).kI);
//                    endTuningDoubles.add(((PIDCoefficients) endTuning[e][i].get(endTuning[e][i])).kD);

                    //check if the kp is not equal
                    if (!endTuningDoubles.get(doubleIndexNumber).equals(TuningStart.initialTuningDoubles.get(doubleIndexNumber))) {
                        telemetry.addData(endTuning[e][i].getName() + " kP", endTuningDoubles.get(doubleIndexNumber));
                        dataExists = true;
                    }
                    doubleIndexNumber++;

                    // check if the ki is not equal
                    if (!endTuningDoubles.get(doubleIndexNumber).equals(TuningStart.initialTuningDoubles.get(doubleIndexNumber))) {
                        telemetry.addData(endTuning[e][i].getName() + " kI", endTuningDoubles.get(doubleIndexNumber));
                        dataExists = true;
                    }
                    doubleIndexNumber++;

                    // check if kd is not equal
                    if (!endTuningDoubles.get(doubleIndexNumber).equals(TuningStart.initialTuningDoubles.get(doubleIndexNumber))) {
                        telemetry.addData(endTuning[e][i].getName() + " kD", endTuningDoubles.get(doubleIndexNumber));
                        dataExists = true;
                    }
                    doubleIndexNumber++;
//                    telemetry.addData("start pidf", TuningStart.initialTuningDoubles);
//                    telemetry.addData("end pidf", endTuningPIDF);
                    /*
                    endTuningPIDF.add((PIDCoefficients) endTuning[e][i].get(endTuning[e][i]));

                    // Check if the pidf is not equal

                    if (endTuningPIDF.get(pidfIndexNumber).equals(TuningStart.initialTuningPIDF.get(pidfIndexNumber))) {
//                            telemetry.addData(endTuning[e].getName(), endTuningPIDF.get(stringIndexNumber));

                        //check if the kp is not equal
                        if (endTuningPIDF.get(pidfIndexNumber).kP == TuningStart.initialTuningPIDF.get(pidfIndexNumber).kP) {
                            telemetry.addData(endTuning[e][i].getName() + " kP", endTuningPIDF.get(pidfIndexNumber).kP);
                        }

                        // check if the ki is not equal
                        if (endTuningPIDF.get(pidfIndexNumber).kI == TuningStart.initialTuningPIDF.get(pidfIndexNumber).kI) {
                            telemetry.addData(endTuning[e][i].getName() + " kI", endTuningPIDF.get(pidfIndexNumber).kI);
                        }

                        // check if kd is not equal
                        if (endTuningPIDF.get(pidfIndexNumber).kD == TuningStart.initialTuningPIDF.get(pidfIndexNumber).kD) {
                            telemetry.addData(endTuning[e][i].getName() + " kD", endTuningPIDF.get(pidfIndexNumber).kD);
                        }
                        telemetry.addData("start pidf", TuningStart.initialTuningPIDF);
                        telemetry.addData("end pidf",endTuningPIDF);

                    }
                    pidfIndexNumber++;

                     */
                } catch (Exception ignored) {

                }
                try {
                    endTuningDoubles.add(((PIDFCoefficients) endTuning[e][i].get(endTuning[e][i])).p);
                    endTuningDoubles.add(((PIDFCoefficients) endTuning[e][i].get(endTuning[e][i])).i);
                    endTuningDoubles.add(((PIDFCoefficients) endTuning[e][i].get(endTuning[e][i])).d);
                    endTuningDoubles.add(((PIDFCoefficients) endTuning[e][i].get(endTuning[e][i])).f);

                    //check if the p is not equal
                    if (!endTuningDoubles.get(doubleIndexNumber).equals(TuningStart.initialTuningDoubles.get(doubleIndexNumber))) {
                        telemetry.addData(endTuning[e][i].getName() + " p", endTuningDoubles.get(doubleIndexNumber));
                        dataExists = true;
                    }
                    doubleIndexNumber++;

                    // check if the i is not equal
                    if (!endTuningDoubles.get(doubleIndexNumber).equals(TuningStart.initialTuningDoubles.get(doubleIndexNumber))) {
                        telemetry.addData(endTuning[e][i].getName() + " i", endTuningDoubles.get(doubleIndexNumber));
                        dataExists = true;
                    }
                    doubleIndexNumber++;

                    // check if d is not equal
                    if (!endTuningDoubles.get(doubleIndexNumber).equals(TuningStart.initialTuningDoubles.get(doubleIndexNumber))) {
                        telemetry.addData(endTuning[e][i].getName() + " d", endTuningDoubles.get(doubleIndexNumber));
                        dataExists = true;
                    }
                    doubleIndexNumber++;
                    // check if f is not equal
                    if (!endTuningDoubles.get(doubleIndexNumber).equals(TuningStart.initialTuningDoubles.get(doubleIndexNumber))) {
                        telemetry.addData(endTuning[e][i].getName() + " f", endTuningDoubles.get(doubleIndexNumber));
                        dataExists = true;
                    }
                    doubleIndexNumber++;
                }
                catch (Exception ignored) {

                }
            }
        }
        if (!dataExists) {
            telemetry.addLine("There is no tuning data that has been changed and recorded :(");
        }

//        telemetry.addData("hi", endTuningDoubles);
        /*
        for (int i=0; i< endTuning3doubles.size(); i++) {
            if (!endTuning3doubles.get(i).equals(TuningStart.initialTuning3doubles.get(i))) {
                telemetry.addData(endTuning3[e].getName(), endTuning3doubles.get(i));
            }
        }

         */



//        telemetry.addData("hi", endTuning3doubles);
    }

    @Override
    public void loop() {
    }
}