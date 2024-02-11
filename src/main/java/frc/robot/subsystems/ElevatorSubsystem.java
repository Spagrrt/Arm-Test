package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.math.ElevatorMath;
import frc.robot.math.Mth;
import frc.robot.math.PID;

import static frc.robot.Constants.*;

public class ElevatorSubsystem extends SubsystemBase {

    ElevatorMath elevatorMath;

    private CANSparkMax rotateMotor1, rotateMotor2, extendMotor, ankleMotor;

    private DutyCycleEncoder rotateEncoder, extendEncoder, ankleEncoder;

    private PID rotatePID, extendPID, anklePID;

    private double desiredExtensionRotations, desiredArmRotations;

    public ElevatorSubsystem() {
        ShuffleboardTab tab = Shuffleboard.getTab("ARM");

        elevatorMath = new ElevatorMath(cArmInit, cArmMinLength, cArmMaxLength, cArmOrigin, cElevatorBoundaries);

        rotateMotor1 = new CANSparkMax(rotateMotor1ID, CANSparkLowLevel.MotorType.kBrushless);
        rotateMotor2 = new CANSparkMax(rotateMotor2ID, CANSparkLowLevel.MotorType.kBrushless);
        extendMotor = new CANSparkMax(extendMotorID, CANSparkLowLevel.MotorType.kBrushless);
        ankleMotor = new CANSparkMax(ankleMotorID, CANSparkLowLevel.MotorType.kBrushless);

        rotateMotor1.setIdleMode(CANSparkBase.IdleMode.kBrake);
        rotateMotor2.setIdleMode(CANSparkBase.IdleMode.kBrake);
        extendMotor.setIdleMode(CANSparkBase.IdleMode.kBrake);
        ankleMotor.setIdleMode(CANSparkBase.IdleMode.kBrake);

        rotateMotor1.setInverted(true);
        rotateMotor2.setInverted(true);
        extendMotor.setInverted(true);
        ankleMotor.setInverted(false);

        rotateEncoder = new DutyCycleEncoder(rotateEncoderID);
        extendEncoder = new DutyCycleEncoder(extendEncoderID);
        ankleEncoder = new DutyCycleEncoder(ankleEncoderID);

        rotatePID = new PID(cRotateP, cRotateI, cRotateD, cRotateMax, cRotateMin, cRotateDeadband, this::getRotation);
        extendPID = new PID(cExtendP, cExtendI, cExtendD, cExtendMax, cExtendMin, cExtendDeadband, this::getExtension);

        setExtensionHome();
        setArmHome();

        tab.add("Rotation Encoder", rotateEncoder);
        tab.add("Extension Encoder", extendEncoder);
        tab.add("Ankle Encoder", ankleEncoder);

        tab.addNumber("Rotation", this::getRotation);
        tab.addNumber("Extension", this::getExtension);
        tab.addNumber("Ankle", this::getAnkle);

        tab.addNumber("Absolute Rotation", this::getRotationAbsolute);
        tab.addNumber("Absolute Extension", this::getExtensionAbsolute);
        tab.addNumber("Ankle Absolute", this::getAnkleAbsolute);
    }

    @Override
    public void periodic() {
        super.periodic();
        rotatePID.setGoal(desiredArmRotations);
        extendPID.setGoal(desiredExtensionRotations);

        double rotatePIDOut = rotatePID.calculate();
        double rotateSpeed = Mth.clamp(rotatePIDOut, -0.8, 0.8);

        System.out.println("Current Rotations: " + getRotation());
        System.out.println("Desired Rotations: " + desiredArmRotations);
        System.out.println("Rotate PID: " + rotatePIDOut);
        System.out.println("Rotate PID Clamped: " + rotateSpeed);

        setRotateSpeed(rotateSpeed);
        setExtendSpeed(Mth.clamp(extendPID.calculate(), -0.8, 0.1));
    }

    public void setDesiredPosition(Translation2d target){
        elevatorMath.setTarget(target);
    }

    public void setDesiredExtension(double desiredExtensionMeters){
        //14cm = 0.454 to -0.84 (-1.294)
        //23.25cm = 0.454 to -1.73 (-2.184)
        //31.5cm = 0.454 to -2.16 (-2.614)

        desiredExtensionRotations = (-9.2686 * desiredExtensionMeters) + 0.3799;

    }

    public void setExtensionHome(){
        desiredExtensionRotations = 0.546;
    }

    public void setDesiredArmAngle(double armAngleRAD){
        desiredArmRotations = (0.1502 * armAngleRAD) + 0.6114;
    }

    public void setArmHome(){
        desiredArmRotations = 0.61;
    }

    public void setRotateSpeed(double speed){
        rotateMotor1.set(speed);
        rotateMotor2.set(speed);
    }

    public void setExtendSpeed(double speed){
        extendMotor.set(speed);
    }

    public void setAnkleSpeed(double speed){
        ankleMotor.set(speed);
    }

    public double getRotation(){
        return rotateEncoder.get();
    }
    public double getExtension(){
        return extendEncoder.get();
    }
    public double getAnkle(){
        return ankleEncoder.get();
    }

    public double getRotationAbsolute(){
        return rotateEncoder.getAbsolutePosition();
    }
    public double getExtensionAbsolute(){
        return extendEncoder.getAbsolutePosition();
    }
    public double getAnkleAbsolute(){
        return ankleEncoder.getAbsolutePosition();
    }

}
