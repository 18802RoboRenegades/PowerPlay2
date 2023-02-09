package org.firstinspires.ftc.teamcode.UltimateGoalCode.BaseCode;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class DriveCode
{
    // Drive motors
    private DcMotor LeftFrontWheelMotor;
    private DcMotor RightFrontWheelMotor;
    private DcMotor LeftRearWheelMotor;
    private DcMotor RightRearWheelMotor;

    public double AutonomousDrivingPower = .3;

    public void init (HardwareMap hardwareMap)
    {
        // Defining drive motors
        LeftFrontWheelMotor = hardwareMap.get(DcMotor.class, "leftFront");
        RightFrontWheelMotor = hardwareMap.get(DcMotor.class, "rightFront");
        LeftRearWheelMotor = hardwareMap.get(DcMotor.class, "leftRear");
        RightRearWheelMotor = hardwareMap.get(DcMotor.class, "rightRear");

        // Setting Motor Power
        LeftFrontWheelMotor.setPower(0);
        RightFrontWheelMotor.setPower(0);
        LeftRearWheelMotor.setPower(0);
        RightRearWheelMotor.setPower(0);

        // Motor Mode
        LeftFrontWheelMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RightFrontWheelMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LeftRearWheelMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RightRearWheelMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Motor ZeroPowerBehavior
        LeftFrontWheelMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RightFrontWheelMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LeftRearWheelMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RightRearWheelMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);




    }

}