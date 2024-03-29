package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.*;

public class ShooterSubsystem extends SubsystemBase {

    CANSparkMax shooterMotor1, shooterMotor2;
    WPI_TalonSRX shooterControlMotor;

    public ShooterSubsystem() {
        shooterMotor1 = new CANSparkMax(shooterMotor1ID, CANSparkLowLevel.MotorType.kBrushless);
        shooterMotor2 = new CANSparkMax(shooterMotor2ID, CANSparkLowLevel.MotorType.kBrushless);

        shooterMotor1.setIdleMode(CANSparkBase.IdleMode.kBrake);
        shooterMotor2.setIdleMode(CANSparkBase.IdleMode.kBrake);

        shooterMotor1.setInverted(false);
        shooterMotor2.setInverted(true);


        shooterControlMotor = new WPI_TalonSRX(shooterControlMotorID);

        shooterControlMotor.setNeutralMode(NeutralMode.Brake);
    }

    public void setShooterSpeed(double speed){
        shooterMotor1.set(speed);
        shooterMotor2.set(speed);
    }

    public void setShooterControlSpeed(double speed){
        shooterControlMotor.set(speed);
    }
}
