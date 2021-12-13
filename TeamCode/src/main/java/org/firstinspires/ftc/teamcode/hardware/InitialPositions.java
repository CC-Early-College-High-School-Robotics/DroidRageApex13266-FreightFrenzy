package org.firstinspires.ftc.teamcode.hardware;

import org.firstinspires.ftc.teamcode.hardware.Devices;

import java.lang.reflect.Field;
import java.util.Arrays;
import java.util.List;

public class InitialPositions {
    public static void output() {
        Field[] doubles = Devices.class.getFields();
        Field[] doubleTwo = Devices.class.getFields();
        Field[] doubleDifference = new Field[1]; //1 is the amount of objects i think maybe im new to this
        boolean hi = Arrays.equals(doubles, doubleTwo);

        // TODO //The optimized method of using the for loop - also called the foreach loop
        //    for (String strTemp : arrData){
        //      System.out.println(strTemp);
        //    }
        for(int i = 0; i < doubles.length; i++){
            boolean hiTwo = doubles[i].equals(doubleTwo[i]); // one thing to consider is that maybe none of the objects will be considered equal becuase they are in differnet classes. if that happens then we should compare the values or something
            if (hiTwo = true) {
                doubleDifference[0] = doubleTwo[i];
            }
            //doubleDifference.getDouble(doubleTwo[i])
        }
        /*
        for(int i = 0; i < doubleDifference.length; i++) {
            doubleDifference[i].getDouble(Object obj)
        }
         */
        // this is from tankdriveroadrunner aka sample tank drive
        //
        // List<Double> velocities = TankKinematics.robotToWheelVelocities(driveSignal.getVel(), TRACK_WIDTH);
        //        List<Double> accelerations = TankKinematics.robotToWheelVelocities(driveSignal.getAccel(), TRACK_WIDTH);
        //        List<Double> feedforwards = Kinematics.calculateMotorFeedforward(velocities, accelerations, kV, kA, kStatic);
        //        leftDriveVeloPID.setPID(LEFT_DRIVE_PID.kP, LEFT_DRIVE_PID.kI, LEFT_DRIVE_PID.kD);
        //        rightDriveVeloPID.setPID(RIGHT_DRIVE_PID.kP, RIGHT_DRIVE_PID.kI, RIGHT_DRIVE_PID.kD);
        //        double leftOutput = feedforwards.get(0) + leftDriveVeloPID.calculate(getWheelVelocities().get(0), velocities.get(0));
        //        double rightOutput = feedforwards.get(1) + leftDriveVeloPID.calculate(getWheelVelocities().get(1), velocities.get(1));
        //        setMotorPowers(leftOutput, rightOutput);
    }
}
