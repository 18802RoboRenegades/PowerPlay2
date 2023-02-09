package org.firstinspires.ftc.teamcode.PowerPlayCode;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.CRServoImpl;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Hardware3 {
    // Drive motors
    public DcMotor leftFrontWheelMotor;
    public DcMotor rightFrontWheelMotor;
    public DcMotor leftRearWheelMotor;
    public DcMotor rightRearWheelMotor;
    public DcMotor elevatorMotor;
    public Servo clawServo;


    public BNO055IMU gyro;



    final public double ROTATION_POWER = 0.4;
    final public double MAX_DRIVING_POWER = 0.75;
    final public double CAROUSEL_POWER = 0.7;
    final public double TICKS_PER_INCH = 16;
    final public double AUTONOMOUS_DRIVING_POWER = 0.5;
    public int TOP_JUNCTION_TICK_VALUE = -4000;
    public int MIDDLE_JUNCTION_TICK_VALUE = -2840;
    public int BOTTOM_JUNCTION_TICK_VALUE = -1620;
    public int FLOOR_TICK_VALUE = 0;
    public double GOING_UP_SPEED = 1;
    public double GOING_DOWN_SPEED = -0.6;
    public int ELEVATOR_TOP_CLIP = -4000;
    public int ELEVATOR_BOTTOM_CLIP = 0;

    public void init (HardwareMap hardwareMap)
    {
        // Defining motors and servos
        leftFrontWheelMotor = hardwareMap.get(DcMotor.class, "leftFront");
        rightFrontWheelMotor = hardwareMap.get(DcMotor.class, "rightFront");
        leftRearWheelMotor = hardwareMap.get(DcMotor.class, "leftRear");
        rightRearWheelMotor = hardwareMap.get(DcMotor.class, "rightRear");
        elevatorMotor = hardwareMap.get(DcMotor.class, "elevatorMotor");
        clawServo = hardwareMap.get(Servo.class, "clawServo");

        //Defining sensors
        gyro = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "gyro.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "gyro";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        gyro.initialize(parameters);

        // Setting Motor Power so the motors don't go crazy when initializing
        leftFrontWheelMotor.setPower(0);
        rightFrontWheelMotor.setPower(0);
        leftRearWheelMotor.setPower(0);
        rightRearWheelMotor.setPower(0);

        rightFrontWheelMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFrontWheelMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRearWheelMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRearWheelMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // Motor Mode
        leftFrontWheelMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontWheelMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRearWheelMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRearWheelMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        elevatorMotor.setTargetPosition(0);
        elevatorMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elevatorMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Motor ZeroPowerBehavior
        leftFrontWheelMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontWheelMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRearWheelMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRearWheelMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // setting Direction
        // USE THIS TO CHANGE MOTOR DIRECTION OVER ALL FUNCTIONS!!
        leftFrontWheelMotor.setDirection(DcMotor.Direction.REVERSE);
        leftRearWheelMotor.setDirection(DcMotor.Direction.REVERSE);
        rightFrontWheelMotor.setDirection(DcMotor.Direction.FORWARD);
        rightRearWheelMotor.setDirection(DcMotor.Direction.FORWARD);

        // setting the select servos to position 0
        //clawServo.setPosition(0.5);

        // init code

    }
}
