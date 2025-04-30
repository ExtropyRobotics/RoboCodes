package pedroPathing.teleOP;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class ArmControler {
    double rotateAngle;
    private DcMotor slider;
    private DcMotor axDown;
    private DcMotor axUp;
    private Servo wrist;
    private Servo claw;
    private double pow = 0;
    private int axPoz = 0;
    private int sliderPoz = 0;
    private int axDirection = 0;
    private int sliderDirection = 0;
    private int maxAmps = 2000;
    private int errorMargin = 10;
    private double clawPos = 0;
    private double wristPos = 0;

    public ArmControler(HardwareMap hardwareMap){

        slider = hardwareMap.get(DcMotor.class, "brat");
        axDown = hardwareMap.get(DcMotor.class, "axDown");
        axUp = hardwareMap.get(DcMotor.class, "axUp");
        wrist = hardwareMap.get(Servo.class, "wrist");
        claw = hardwareMap.get(Servo.class ,"claw");

        initializeMotor(axUp, DcMotorSimple.Direction.REVERSE);
        initializeMotor(axDown, DcMotorSimple.Direction.FORWARD);
        initializeMotor(slider, DcMotorSimple.Direction.REVERSE);
    }
    public void setPower(double pow){
        this.pow = pow;
        axUp.setPower(pow);
        axDown.setPower(pow);
    }
    public void setPowerSlider(double pow){
        this. pow = pow;
        slider.setPower(pow);
    }
    public int setAxPoz (int targetPozAx){
        if(targetPozAx - axPoz > 5) axDirection = 1;
        if(targetPozAx - axPoz < -5) axDirection = -1;

        axPoz = targetPozAx;
        axUp.setTargetPosition(axPoz);
        axDown.setTargetPosition(axPoz);

        axDown.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        axUp.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        return axDirection;
    }
    public int setSliderPoz (int targetPozSlider){
        if(targetPozSlider - slider.getCurrentPosition() > 5) sliderDirection = 1;
        if(targetPozSlider - slider.getCurrentPosition() < -5) sliderDirection = -1;

        if(sliderPoz > slider.getCurrentPosition()-errorMargin && sliderPoz < slider.getCurrentPosition()+errorMargin)
            slider.setPower(0);
        else slider.setPower(pow);

        sliderPoz = targetPozSlider;
        slider.setTargetPosition(sliderPoz);

        slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        return sliderDirection;
    }
    public void setWrist(double pos){
        wrist.setPosition(pos);
        wristPos = pos;
    }
    public void setClaw(double pos){
        claw.setPosition(pos);
        clawPos = pos;
    }
    public void setWristParalel(double offset, double multiplier){
        rotateAngle = axUp.getCurrentPosition()/900.0 * 360;
        double pos = -rotateAngle / 500 * (0-0.6191) * multiplier + offset;
        setWrist(pos);
    }
    private void initializeMotor(DcMotor motor, DcMotorSimple.Direction direction){
        motor.setDirection(direction);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setTargetPosition(0);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ((DcMotorEx)axUp).setCurrentAlert(maxAmps, CurrentUnit.MILLIAMPS);
    }
}