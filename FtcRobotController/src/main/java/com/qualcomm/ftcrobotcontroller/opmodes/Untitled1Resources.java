package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.hardware.DcMotorController;

/**
 * Created by Stephen on 1/12/2016.
 */
public class Untitled1Resources {

    // Enter the number of Ticks per single Rotation of Motor (ours is 1440)
    final static int EncoderTicksPerRotation = 1440;

    // Calculated gear ratio from motor to wheel
    // Number of Teeth on the Tire Axle Gear divided by the Number of Teeth on the Motor Gear
    final static double GEAR_RATIO = 2;

    // The Full Diameter of the Wheel Tread in Inches
    // Normally marked on the side of the Tire Tread, but might not be in inches.
    final static int WheelDiameter = 4;

    // Calculating the Circumference of the Tire based on the above Variables Provided
    final static double WHEEL_CIRCUMFERENCE = Math.PI * WheelDiameter;

    //The distance between the left and right tires
    final static double AxleWidth = 15.5;

    //Add a buffer to the inside turn radius to prevent freezing
    final static double AxleWidthBuffer = 1;
    final static double DISTANCEturnx = 0.50;
    final static double DISTANCEturn1 = 12.964;
    final static double DISTANCEfoward1 = 85.00;
    final static double DISTANCEturn2 = 38.893; //135 degrees
    final static double DISTANCEback1 = 23.00;

    /**
     * This Method will calculate the number of COUNTS for the motor to turn
     * based on the given parameter of distanceInches when called
     */
    public double driveInchesForEncoder (double distanceInches) {
        final double ROTATIONS = distanceInches / WHEEL_CIRCUMFERENCE;
        final double COUNTS = EncoderTicksPerRotation * ROTATIONS * GEAR_RATIO;
        return COUNTS;
    }

    public static double turnForDegrees (double Degrees, String WheelLocation) {

        final double WheelCircumference = WheelDiameter * Math.PI;

        if (WheelLocation == "Outer") {
            final double TurnDiameterOuter = AxleWidth + AxleWidthBuffer * 2;
            final double TurnCircumferenceOuter = TurnDiameterOuter * Math.PI;
            final double TickstoTurnOuter = TurnCircumferenceOuter / WheelDiameter * EncoderTicksPerRotation;
            final double TurnTicksOuter = Degrees / TickstoTurnOuter;
            return (TurnTicksOuter);
        }
        if (WheelLocation == "Inner") {

            final double TurnDiameterInner = AxleWidthBuffer * 2;
            final double TurnCircumferenceInner = TurnDiameterInner * Math.PI;
            final double TickstoTurnInner = TurnCircumferenceInner / WheelDiameter * EncoderTicksPerRotation;
            final double TurnTicksInner = Degrees / TickstoTurnInner;
            return (TickstoTurnInner);
        } else {
            //Do Nothing
            return 0;
        }
    }
       /* public double TurnRobot ()
    {

        final double leftMotorTicks_turn2 = turnForDegrees(135, "Inner");
        final double rightMotorTicks_turn2 = turnForDegrees(135, "Outer");

        leftMotor.setTargetPosition((int) leftMotorTicks_turn2 );
        rightMotor.setTargetPosition((int) rightMotorTicks_turn2);

        leftMotor.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
        rightMotor.setMode(DcMotorController.RunMode.RUN_TO_POSITION);

        leftMotor.setPower(-1.0);
        rightMotor.setPower(-0.2);

        if (Math.abs (leftMotor.getCurrentPosition ()) > leftMotorTicks_turn2)
        {
            leftMotor.setPower(0.0);
            leftMotor.setMode(DcMotorController.RunMode.RESET_ENCODERS);
            boolean leftMotorDone = true;
        }
        if (Math.abs (rightMotor.getCurrentPosition ()) > rightMotorTicks_turn2)
        {
            rightMotor.setPower(0.0);
            leftMotor.setMode(DcMotorController.RunMode.RESET_ENCODERS);
            boolean rightMotorDone = true;
        }
        if ((leftMotorDone) && (rightMotorDone)){
            boolean leftMotorDone = false;
            boolean rightMotorDone = false;
            botstate = State.back2;
        }




    }
*/

}
