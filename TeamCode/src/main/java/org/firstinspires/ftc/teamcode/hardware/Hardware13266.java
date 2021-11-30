/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 * Motor channel:  Manipulator drive motor:  "left_arm"
 * Servo channel:  Servo to open left claw:  "left_hand"
 * Servo channel:  Servo to open right claw: "right_hand"
 */
public class Hardware13266 {
    /* Public OpMode members. */

    // Drivetrain
    public DcMotorEx leftFront = null;
    public DcMotorEx leftRear = null;
    public DcMotorEx rightFront = null;
    public DcMotorEx rightRear = null;

    // Cycles variable (to calculate frequency)
    public int cycles = 0;

    // Slow mode variable for slow mode
    public double slowMode = 1;

    // Intake Speed
    public final double INTAKE_VELOCITY = 1000;

    // Carousel Speed
    public final double CAROUSEL_POWER = 0.4;

    // Box servo positions
    public final double BOX_UP = 0.65;
    public final double BOX_INTAKE = 1.0;
    public final double BOX_DROP = 0.19;

    // Arm Positions
    public final double ARM_HIGH_POS = 1.5;
    public final double ARM_MID_POS = 0.90;
    public final double ARM_LOW_POS = 0.4;
    public final double ARM_INTAKE_POS = -0.3;
    public final double ARM_POWER = 0.5;

    public void setArmPosition(double pos){
        armMotor.setTargetPosition((int) (TICKS_PER_REV * pos));
    }
    public double getArmPosition(){
        return armMotor.getCurrentPosition()/TICKS_PER_REV;
    }

    // Arm Motor
    public DcMotorEx armMotor = null;

    // Duck Motor
    public DcMotorEx carouselMotor = null;

    // Intake Motor
    public DcMotorEx intakeMotor = null;

    // Servos

    public Servo boxServo = null;

    // Constants
    public final double TICKS_PER_REV = 383.6;

    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    public ElapsedTime runtime  = new ElapsedTime();

    /* Constructor */
    public Hardware13266(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).

        // Drivetrain Initialization
        leftFront = hwMap.get(DcMotorEx.class, "leftFront");
        leftRear = hwMap.get(DcMotorEx.class, "leftRear");
        rightFront  = hwMap.get(DcMotorEx.class, "rightFront");
        rightRear  = hwMap.get(DcMotorEx.class, "rightRear");

        // Arm Motor Initialization
        armMotor = hwMap.get(DcMotorEx.class,"armMotor");

        // Carousel Motor

        carouselMotor = hwMap.get(DcMotorEx.class,"carouselMotor");

        // Intake Motor

        intakeMotor = hwMap.get(DcMotorEx.class,"intakeMotor");

        // Servo Initialization
        boxServo = hwMap.get(Servo.class,"boxServo");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery

        // Drivetrain Motor Directions
        leftFront.setDirection(DcMotorEx.Direction.FORWARD);
        leftRear.setDirection(DcMotorEx.Direction.FORWARD);
        rightFront.setDirection(DcMotorEx.Direction.REVERSE);
        rightRear.setDirection(DcMotorEx.Direction.REVERSE);

        // Arm Motor Directions
        armMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Duck Motor Directions
        carouselMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        // Intake Motor Directions
        carouselMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        // Servo Directions
        boxServo.setDirection(Servo.Direction.FORWARD);

        // Intake Direcitons
        intakeMotor.setDirection(DcMotorEx.Direction.REVERSE);
        // Adjusting the Zero Power Behavior changes how the motors behaved when a
        // Power of 0 is applied.

        // Drivetrain Zero Power Behavior
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Arm Motor Zero Power Behavior
        armMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        // Duck Motor Zero Power Behavior

        carouselMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Duck Motor Zero Power Behavior

        carouselMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Duck Motor Zero Power Behavior

        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Turn on Run Using Encoder to use the built in PID controller


        // Arm Encoders
        armMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        armMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        armMotor.setTargetPosition(0);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Arm Encoders
        intakeMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        intakeMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        //carouselMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        //carouselMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        // Drivetrain Encoders
        /*
        leftFront.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        leftRear.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rightRear.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        // Reset Encoders before starting OpMode
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        */

    }
}

