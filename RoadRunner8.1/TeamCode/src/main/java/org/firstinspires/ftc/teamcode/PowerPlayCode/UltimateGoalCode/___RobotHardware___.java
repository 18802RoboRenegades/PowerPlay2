package org.firstinspires.ftc.teamcode.UltimateGoalCode;

import com.qualcomm.robotcore.hardware.CRServoImpl;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

// 44 motor ticks to 1 inch for the old motors.

public class ___RobotHardware___
{
    // Motors and Servos
    private DcMotor LeftFrontWheelMotor;
    private DcMotor RightFrontWheelMotor;
    private DcMotor LeftRearWheelMotor;
    private DcMotor RightRearWheelMotor;
    private Servo ShooterServo2;
    private DcMotor ShooterMotor1;
    private DcMotor ShooterMotor2;
    private DcMotor WobbleMotor;
    private Servo WobbleServo;
    private DcMotor IntakeMotor;
    private Servo FeedGateServo;
    private CRServoImpl IntakeServo;

    // some methods for the motors and servos
    private double intakeLoadPower = .57;
    private double intakeUnLoadPower = -.57;
    private double shooterPower = .595;
    private double powerShotShooterPower = .45;
    private double triggerPosition = .2;
    private double maxDrivingPower = 1;


    private int Shooter1Position = 0;
    private int Shooter2Position = 0;
    private long previousTime = 0;
    //private long currentTime = 0;
    private double Shooter1RPM = 0;
    private double Shooter2RPM = 0;
    private int SHOOTER_PPR_OUTPUT = 28;
    public int DesiredShooterRPM = 3300;
    private int POWER_SHOT_RPM = 2500;
    private int HIGH_GOAL_RPM = 3300;


    public boolean isHandOpen = false;
    public boolean isVulcanOn = false;

    // some methods for autonomous
    public double AutonomousDrivingPower = .3;
    public double AutoVulcanPower = .65;
    public int AutoDesiredShooterRPM = 2400;
    private int AUTO_HIGH_GOAL_RPM = 3600;
    private int AUTO_POWER_SHOT_RPM = 2400;


    public void init(HardwareMap hardwareMap)
    {
        // This code is defining the motors and servos.
        // You can also look at this code to see what the motors and servos will appear on the phone.
        LeftFrontWheelMotor = hardwareMap.get(DcMotor.class, "leftFront");
        RightFrontWheelMotor = hardwareMap.get(DcMotor.class, "rightFront");
        LeftRearWheelMotor = hardwareMap.get(DcMotor.class, "leftRear");
        RightRearWheelMotor = hardwareMap.get(DcMotor.class, "rightRear");
        ShooterServo2 = hardwareMap.get(Servo.class, "shooterServo2");
        ShooterMotor1 = hardwareMap.get(DcMotor.class, "shooterMotor1");
        ShooterMotor2 = hardwareMap.get(DcMotor.class, "shooterMotor2");
        WobbleMotor = hardwareMap.get(DcMotor.class, "wobbleMotor");
        WobbleServo = hardwareMap.get(Servo.class, "wobbleServo");
        IntakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        IntakeServo = hardwareMap.get(CRServoImpl.class, "IntakeServo");
        FeedGateServo = hardwareMap.get(Servo.class, "feedGateServo");


        // Setting Motor Power
        LeftFrontWheelMotor.setPower(0);
        RightFrontWheelMotor.setPower(0);
        LeftRearWheelMotor.setPower(0);
        RightRearWheelMotor.setPower(0);
        ShooterMotor1.setPower(0);
        ShooterMotor2.setPower(0);
        WobbleMotor.setPower(0);
        IntakeMotor.setPower(0);
        IntakeServo.setPower(0);

        // Setting Motor Mode
        LeftFrontWheelMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RightFrontWheelMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LeftRearWheelMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RightRearWheelMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ShooterMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ShooterMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        WobbleMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        IntakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Setting Motor zeroPowerBehavior
        LeftFrontWheelMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RightFrontWheelMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LeftRearWheelMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RightRearWheelMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ShooterMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ShooterMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        WobbleMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        IntakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Setting Servo Position
        ShooterServo2.setPosition(0);
        WobbleServo.setPosition(0);
        FeedGateServo.setPosition(0);
    }
    // These are just some variables.
    public void LoadMagazine()
    {
        this.IntakeMotor.setPower(intakeLoadPower);
    }
    public void UnLoadMag()
    {
        this.IntakeMotor.setPower(intakeUnLoadPower);
    }
    public void StopMag()
    {
        this.IntakeMotor.setPower(0);
    }
    public void FireDisk()
    {
        this.ShooterServo2.setPosition(triggerPosition);
    }
    public void ResetTrigger()
    {
        this.ShooterServo2.setPosition(0);
    }
    public void StartVulcan()
    {
        if(!isVulcanOn)
        {
            this.ShooterMotor1.setPower(shooterPower);
            this.ShooterMotor2.setPower(-shooterPower + .03);
            isVulcanOn = true;
        }
    }
    public void StopVulcan()
    {
        if(isVulcanOn)
        {
            this.ShooterMotor1.setPower(0);
            this.ShooterMotor2.setPower(0);
            isVulcanOn = false;
        }
    }
    public void MoveWobble(float y)
    {
        WobbleMotor.setPower(Range.clip(y, -.75, .75));
    }
    public void OpenWobbleHand()
    {
        this.WobbleServo.setPosition(.3);
        isHandOpen = true;
    }
    public void CloseWobbleHand()
    {
        this.WobbleServo.setPosition(0);
        isHandOpen = false;
    }
    public void PropDisk()
    {
        this.IntakeServo.setPower(-1);
    }
    public void StopProp()
    {
        this.IntakeServo.setPower(0);
    }
    public void ReverseProp()
    {
        this.IntakeServo.setPower(1);
    }
    public void DropFeedGate()
    {
        this.FeedGateServo.setPosition(.45);
    }

    public boolean PrecisionDriveForwardReverse(int inches)
    {
        // 1 tick = .44 inches.
        // cir = 12.3 in
        // ticksPerInch was 44

        int ticksPerInch = 31;

        this.LeftFrontWheelMotor.getCurrentPosition();
        this.RightFrontWheelMotor.getCurrentPosition();
        this.LeftRearWheelMotor.getCurrentPosition();
        this.RightRearWheelMotor.getCurrentPosition();

        this.LeftFrontWheelMotor.setTargetPosition(inches * ticksPerInch);
        this.LeftRearWheelMotor.setTargetPosition(-inches * ticksPerInch);
        this.RightFrontWheelMotor.setTargetPosition(inches * ticksPerInch);
        this.RightRearWheelMotor.setTargetPosition(-inches * ticksPerInch);

        this.LeftFrontWheelMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.LeftRearWheelMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.RightFrontWheelMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.RightRearWheelMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        LeftFrontWheelMotor.setPower(AutonomousDrivingPower);
        LeftRearWheelMotor.setPower(AutonomousDrivingPower);
        RightFrontWheelMotor.setPower(AutonomousDrivingPower);
        RightRearWheelMotor.setPower(AutonomousDrivingPower);



        while(this.LeftRearWheelMotor.isBusy());
        this.RightFrontWheelMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.LeftFrontWheelMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.RightRearWheelMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.LeftRearWheelMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        return true;
    }
    public boolean PrecisionStrafe(int inches)
    {
        // 1 tick = .44 inches.
        // cir = 12.3 in
        // ticksPerInch was 88
        // ticksPerInch was 83

        int ticksPerInch = 62;

        this.LeftFrontWheelMotor.getCurrentPosition();
        this.RightFrontWheelMotor.getCurrentPosition();
        this.LeftRearWheelMotor.getCurrentPosition();
        this.RightRearWheelMotor.getCurrentPosition();

        this.LeftFrontWheelMotor.setTargetPosition(-inches * (ticksPerInch));
        this.LeftRearWheelMotor.setTargetPosition(-inches * ticksPerInch);
        this.RightFrontWheelMotor.setTargetPosition(inches * (ticksPerInch));
        this.RightRearWheelMotor.setTargetPosition(inches * ticksPerInch);

        this.LeftFrontWheelMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.LeftRearWheelMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.RightFrontWheelMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.RightRearWheelMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        LeftFrontWheelMotor.setPower(AutonomousDrivingPower);
        LeftRearWheelMotor.setPower(AutonomousDrivingPower);
        RightFrontWheelMotor.setPower(AutonomousDrivingPower);
        RightRearWheelMotor.setPower(AutonomousDrivingPower);


        while(this.LeftRearWheelMotor.isBusy()
                || this.LeftFrontWheelMotor.isBusy()
                || this.RightFrontWheelMotor.isBusy()
                || this.RightRearWheelMotor.isBusy());
        this.RightFrontWheelMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.LeftFrontWheelMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.RightRearWheelMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.LeftRearWheelMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        return  true;
    }
    public void ResetEncoder()
    {
        this.LeftFrontWheelMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.LeftRearWheelMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.RightFrontWheelMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.RightRearWheelMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public void AutoStartVulcan()
    {
        if(!isVulcanOn)
        {
            this.ShooterMotor1.setPower(AutoVulcanPower);
            this.ShooterMotor2.setPower(-AutoVulcanPower);
            isVulcanOn = true;
        }
        else
        {
            CheckAutoShooterRPM();
        }
    }
    public void AutoRotate(int angle)
    {
        int PPR = 20;
        int numberOfTicks = 0;

        if(Math.abs(angle) == 45)
        {

        }
    }
    public void SetPowerShotRPM()
    {
        DesiredShooterRPM = POWER_SHOT_RPM;
    }
    public void SetHighGoalRPM()
    {
        DesiredShooterRPM = HIGH_GOAL_RPM;
    }
    public void Drive(double drive, double turn, double strafe)
    {
        double leftPower    = -Range.clip(drive + turn, -maxDrivingPower, maxDrivingPower) ;
        double rightPower   = Range.clip(drive - turn, -maxDrivingPower, maxDrivingPower) ;
        double strafePower = Range.clip( strafe, -maxDrivingPower, maxDrivingPower );

        this.LeftFrontWheelMotor.setPower(leftPower - strafePower);
        this.LeftRearWheelMotor.setPower(rightPower - strafePower);
        this.RightFrontWheelMotor.setPower(leftPower + strafePower);
        this.RightRearWheelMotor.setPower(rightPower + strafePower);
    }

    private void CheckShooterRPM() {
        // Setting RPM for Vulcan (Shooter)
        try {
            int positionDelta1 = Math.abs(this.ShooterMotor1.getCurrentPosition()) - Math.abs(this.Shooter1Position);
            long timeDelta1 = System.currentTimeMillis() - this.previousTime;
            if (timeDelta1 ==0 || positionDelta1 == 0) {
                this.ShooterMotor1.setPower(shooterPower);
                this.ShooterMotor2.setPower(-shooterPower);
            }
            else if (timeDelta1 > 500)
            {
                double rValue = positionDelta1 / SHOOTER_PPR_OUTPUT;
                double mValue = (double)timeDelta1 / 60000.0;
                Shooter1RPM = (rValue/mValue);
                if (Shooter1RPM < DesiredShooterRPM - 10) {
                    //Increase Power
                    //double power = this.ShooterMotor1.getPower();
                    this.ShooterMotor1.setPower(this.ShooterMotor1.getPower() + .01);
                    this.ShooterMotor2.setPower(this.ShooterMotor2.getPower() - .01);
                } else if (Shooter1RPM > DesiredShooterRPM + 10) {
                    //Decrease Power
                    this.ShooterMotor1.setPower(this.ShooterMotor1.getPower() - .01);
                    this.ShooterMotor2.setPower(this.ShooterMotor2.getPower() + .01);
                }

                // Set new current position and time
                this.previousTime = System.currentTimeMillis();
                this.Shooter1Position = this.ShooterMotor1.getCurrentPosition();
            }
        }
        catch (Exception exc)
        {
            int i=0;
        }
    }
    private void CheckAutoShooterRPM() {
        // Setting RPM for Vulcan (Shooter)
        try {
            int positionDelta1 = Math.abs(this.ShooterMotor1.getCurrentPosition()) - Math.abs(this.Shooter1Position);
            long timeDelta1 = System.currentTimeMillis() - this.previousTime;
            if (timeDelta1 == 0 || positionDelta1 == 0) {
                this.ShooterMotor1.setPower(AutoVulcanPower);
                this.ShooterMotor2.setPower(-AutoVulcanPower);
            }
            else if (timeDelta1 > 100)
            {
                double rValue = positionDelta1 / SHOOTER_PPR_OUTPUT;
                double mValue = (double)timeDelta1 / 60000.0;
                Shooter1RPM = (rValue/mValue);
                if (Shooter1RPM < AutoDesiredShooterRPM - 10) {
                    // Increase Power
                    this.ShooterMotor1.setPower(this.ShooterMotor1.getPower() + .01);
                    this.ShooterMotor2.setPower(this.ShooterMotor2.getPower() - .01);
                } else if (Shooter1RPM > AutoDesiredShooterRPM + 10) {
                    // Decrease Power
                    this.ShooterMotor1.setPower(this.ShooterMotor1.getPower() - .01);
                    this.ShooterMotor2.setPower(this.ShooterMotor2.getPower() + .01);
                }

                // Set new current position and time
                this.previousTime = System.currentTimeMillis();
                this.Shooter1Position = this.ShooterMotor1.getCurrentPosition();
            }
        }
        catch (Exception exc)
        {
            int i=0;
        }
    }
    public void PowerShotStartVulcan()
    {
        if(!isVulcanOn)
        {

            this.ShooterMotor1.setPower(powerShotShooterPower);
            this.ShooterMotor2.setPower(-powerShotShooterPower - .03);
            isVulcanOn = true;

        }
    }

    public double[] GetShooterRPM()
    {
        try
        {
            int positionDelta1 = Math.abs(this.ShooterMotor1.getCurrentPosition()) - Math.abs(this.Shooter1Position);
            int positionDelta2 = Math.abs(this.ShooterMotor2.getCurrentPosition()) - Math.abs(this.Shooter2Position);
            long timeDelta1 = System.currentTimeMillis() - this.previousTime;
            double rValue1 = positionDelta1 / SHOOTER_PPR_OUTPUT;
            double rValue2 = positionDelta2 / SHOOTER_PPR_OUTPUT;
            double mValue = (double) timeDelta1 / 60000.0;
            // sometimes checking too often will result in invalid values
            if (timeDelta1 > 250)
            {
                Shooter1RPM = (rValue1 / mValue);
                Shooter2RPM = (rValue2 / mValue);

                // Set new current position and time
                this.previousTime = System.currentTimeMillis();
                this.Shooter1Position = this.ShooterMotor1.getCurrentPosition();
                this.Shooter2Position = this.ShooterMotor2.getCurrentPosition();
            }

        }
        catch (Exception exc)
        {
           // Do nothing.  This is just for debugging and protecting the bot from exceptions.
        }

        return new double[]{Shooter1RPM,Shooter2RPM};
    }

}
// yo you're looking at the bottom