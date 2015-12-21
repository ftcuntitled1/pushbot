

package com.qualcomm.ftcrobotcontroller.opmodes;

//------------------------------------------------------------------------------
//
// PushBotAuto
//

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Extends the PushBotTelemetry and PushBotHardware classes to provide a basic
 * autonomous operational mode for the Push Bot.
 *
 * @author Tyler Bechard
 * @version 2015-10-31
 */

public class UntitledAuto extends OpMode{

    final double LEFT_GRIP_OPEN_POSITION = 0.0;
    final double LEFT_GRIP_CLOSED_POSITION = 1.0;
    final double RIGHT_GRIP_OPEN_POSITION = 1.0;
    final double RIGHT_GRIP_CLOSED_POSITION = 0.0;

    final double LEFT_SWEEP_OPEN_POSITION = 0.0;
    final double LEFT_SWEEP_CLOSED_POSITION = 1.0;
    final double RIGHT_SWEEP_OPEN_POSITION = 1.0;
    final double RIGHT_SWEEP_CLOSED_POSITION = 0.0;

    final static int ENCODER_CPR = 1440;
    final static double GEAR_RATIO = 2;
    final static int WHEEL_DIAMETER = 4;
    final static double CIRCUMFERENCE = Math.PI * WHEEL_DIAMETER;

    final static double DISTANCEturn1 = 11.78;
    final static double ROTATIONSturn1 = DISTANCEturn1 / CIRCUMFERENCE;
    final static double COUNTSturn1 = ENCODER_CPR * ROTATIONSturn1 * GEAR_RATIO;

    final static double DISTANCEfoward1 = 85.00;
    final static double ROTATIONSfoward1 = DISTANCEfoward1 / CIRCUMFERENCE;
    final static double COUNTSfoward1 = ENCODER_CPR * ROTATIONSfoward1 * GEAR_RATIO;

    final static double DISTANCEturn2 = 34.2;
    final static double ROTATIONSturn2 = DISTANCEturn2 / CIRCUMFERENCE;
    final static double COUNTSturn2 = ENCODER_CPR * ROTATIONSturn2 * GEAR_RATIO;

    final static double DISTANCEback1 = 23.00;
    final static double ROTATIONSback1 = DISTANCEback1 / CIRCUMFERENCE;
    final static double COUNTSback1 = ENCODER_CPR * ROTATIONSback1 * GEAR_RATIO;

    DcMotor leftArm;

    DcMotor leftMotor;
    DcMotor rightMotor;

    Servo leftGripper;
    Servo rightGripper;

    Servo leftSweeper;
    Servo rightSweeper;


    enum State {startup, turn1, foward1, turn2, lift, back2, drop, flippers, done}

    ElapsedTime time;

    static final double sweeperTime = 1.0;
    static final double dropTime = 1.0;
    static final double liftTime = 1.0;

    State botstate;

    @Override
    public void init () {
        time = new ElapsedTime();

        botstate = State.startup;

        leftGripper = hardwareMap.servo.get("left_hand");
        rightGripper = hardwareMap.servo.get("right_hand");

        leftSweeper = hardwareMap.servo.get("left_sweeper");
        rightSweeper = hardwareMap.servo.get("right_sweeper");

        leftMotor = hardwareMap.dcMotor.get("left_drive");
        rightMotor = hardwareMap.dcMotor.get("right_drive");

        rightMotor.setDirection(DcMotor.Direction.REVERSE);

        leftArm = hardwareMap.dcMotor.get("left_arm");

        leftMotor.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        rightMotor.setMode(DcMotorController.RunMode.RESET_ENCODERS);

        leftGripper.setPosition(LEFT_GRIP_OPEN_POSITION);
        rightGripper.setPosition(RIGHT_GRIP_OPEN_POSITION);

        rightSweeper.setPosition(LEFT_SWEEP_CLOSED_POSITION);
        rightSweeper.setPosition(RIGHT_SWEEP_CLOSED_POSITION);
    }

    @Override
    public void loop () {

        switch(botstate) {
            case startup:
                //reset_drive_encoders ();
                leftMotor.setMode(DcMotorController.RunMode.RESET_ENCODERS);
                rightMotor.setMode(DcMotorController.RunMode.RESET_ENCODERS);

                botstate = State.turn1;

                break;

            case turn1:

                // turn left 45 degrees

                //leftMotor.setTargetPosition((int) COUNTS);
                rightMotor.setTargetPosition((int) COUNTSturn1);

                //leftMotor.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
                rightMotor.setMode(DcMotorController.RunMode.RUN_TO_POSITION);

                //leftMotor.setPower(0.5);
                rightMotor.setPower(0.5);

                if (Math.abs (rightMotor.getCurrentPosition ()) > COUNTSturn1)
                {
                    rightMotor.setPower(0.0);
                    leftMotor.setMode(DcMotorController.RunMode.RESET_ENCODERS);
                    rightMotor.setMode(DcMotorController.RunMode.RESET_ENCODERS);
                    botstate = State.foward1;
                }




                break;

            case foward1:

                // Go forward 105 inches
                // (Including Flippers)

                leftMotor.setTargetPosition((int) COUNTSfoward1);
                rightMotor.setTargetPosition((int) COUNTSfoward1);

                leftMotor.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
                rightMotor.setMode(DcMotorController.RunMode.RUN_TO_POSITION);

                leftMotor.setPower(0.5);
                rightMotor.setPower(0.5);

                if ((Math.abs (rightMotor.getCurrentPosition ()) > COUNTSfoward1) &&
                        (Math.abs (leftMotor.getCurrentPosition ()) > COUNTSfoward1))
                {
                    leftMotor.setPower(0.0);
                    rightMotor.setPower(0.0);
                    leftMotor.setMode(DcMotorController.RunMode.RESET_ENCODERS);
                    rightMotor.setMode(DcMotorController.RunMode.RESET_ENCODERS);
                    botstate = State.turn2;
                }

                break;

            case turn2:

                // Turn left 315 degrees


                leftMotor.setTargetPosition((int) COUNTSturn2);
                //rightMotor.setTargetPosition((int) COUNTSturn2);

                leftMotor.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
                //rightMotor.setMode(DcMotorController.RunMode.RUN_TO_POSITION);

                leftMotor.setPower(0.2);
                //rightMotor.setPower(0.5);

                if (Math.abs (leftMotor.getCurrentPosition ()) > COUNTSturn2)
                {
                    leftMotor.setPower(0.0);
                    leftMotor.setMode(DcMotorController.RunMode.RESET_ENCODERS);
                    rightMotor.setMode(DcMotorController.RunMode.RESET_ENCODERS);
                    botstate = State.back2;
                }

                break;

            case back2:

                // Go backwards 23 inches

                leftMotor.setTargetPosition((int) COUNTSback1);
                rightMotor.setTargetPosition((int) COUNTSback1);

                leftMotor.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
                rightMotor.setMode(DcMotorController.RunMode.RUN_TO_POSITION);

                leftMotor.setPower(-0.5);
                rightMotor.setPower(-0.5);
                if ((Math.abs (rightMotor.getCurrentPosition ()) > COUNTSback1) &&
                        (Math.abs (leftMotor.getCurrentPosition ()) > COUNTSback1))
                {
                    leftMotor.setPower(-0.0);
                    rightMotor.setPower(0.0);
                    leftMotor.setMode(DcMotorController.RunMode.RESET_ENCODERS);
                    rightMotor.setMode(DcMotorController.RunMode.RESET_ENCODERS);
                    botstate = State.lift;
                }

                break;

            case lift:

                // Move arm motor up unknown ticks
                double currentTimelift = time.time();

                if (currentTimelift < liftTime) {
                    //move arm slowly
                    leftArm.setPower(-0.5);
                } else {
                    leftArm.setPower(0.0);
                    botstate = State.drop;
                    }



                break;

            case drop:

                // Drop object
                //(Open Grippers)

                double currentTimedrop = time.time();

                leftGripper.setPosition(LEFT_GRIP_CLOSED_POSITION);
                rightGripper.setPosition(RIGHT_GRIP_CLOSED_POSITION);
                if (currentTimedrop < dropTime) {
                    //do nothing but wait or sleep
                } else {
                    leftGripper.setPosition(LEFT_GRIP_CLOSED_POSITION);
                    rightGripper.setPosition(RIGHT_GRIP_CLOSED_POSITION);
                    botstate = State.done;
                }


                break;

            case flippers:
                // activate flippers (open and close)

                double currentTimeflip = time.time();

                leftSweeper.setPosition(LEFT_SWEEP_OPEN_POSITION);
                leftSweeper.setPosition(RIGHT_SWEEP_OPEN_POSITION);
                if (currentTimeflip > sweeperTime) {
                    //do nothing but wait or sleep
                } else {
                    rightSweeper.setPosition(LEFT_SWEEP_CLOSED_POSITION);
                    rightSweeper.setPosition(RIGHT_SWEEP_CLOSED_POSITION);
                }

                break;

            case done:
                // stop all functions

                break;

            default:
            //
            // The autonomous actions have been accomplished (i.e. the state has
            // transitioned into its final state.
            //
            break;
        }


        // Send telemetry data to the driver station.
        //


    } // PushBotAuto::loop
} // PushBotAuto
