// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Constants.chassis;
import frc.robot.Constants.factor;

public class drivetrain extends SubsystemBase {
  CANSparkMax leftLeader = new CANSparkMax(4, MotorType.kBrushless);
  CANSparkMax rightLeader = new CANSparkMax(2, MotorType.kBrushless);
  CANSparkMax leftFollower = new CANSparkMax(5, MotorType.kBrushless);
  CANSparkMax rightFollower = new CANSparkMax(3, MotorType.kBrushless);

  RelativeEncoder leftEncoder = leftLeader.getEncoder();
  RelativeEncoder rightEncoder = rightLeader.getEncoder();

  private final SysIdRoutine driveRoutine = new SysIdRoutine(
    new SysIdRoutine.Config(
      null,
      Volts.of(7),
      null, 
      null)
      ,new SysIdRoutine.Mechanism(
        (voltage) -> runDrivetrianVolts(voltage.in(Volts)),
        null,this
      ));

  public drivetrain() {
    rightLeader.restoreFactoryDefaults();
    leftLeader.restoreFactoryDefaults();
    rightFollower.restoreFactoryDefaults();
    leftFollower.restoreFactoryDefaults();

    rightLeader.setIdleMode(IdleMode.kBrake);
    leftLeader.setIdleMode(IdleMode.kBrake);
    rightFollower.setIdleMode(IdleMode.kBrake);
    leftFollower.setIdleMode(IdleMode.kBrake);

    rightLeader.setSmartCurrentLimit(40);
    leftLeader.setSmartCurrentLimit(40);
    rightFollower.setSmartCurrentLimit(40);
    leftFollower.setSmartCurrentLimit(40);

    rightLeader.setInverted(true);
    leftLeader.setInverted(false);

    rightFollower.follow(rightLeader, false);
    leftFollower.follow(leftLeader, false);

    resetEncoders();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("distance", getMeters());
    SmartDashboard.putNumber("velocity", getVelocity());
    SmartDashboard.putNumber("volts", getVolts());

    SmartDashboard.putNumber("Left Encoder Pos", getLeft());
    SmartDashboard.putNumber("Right Encoder Pos", getRight());

  }

  public void runDrivetrianVolts(double volts){
    leftLeader.setVoltage(volts);
    rightLeader.setVoltage(volts);
  }

  public void resetEncoders(){
    leftEncoder.setPosition(0);
    rightEncoder.setPosition(0);
  }

  public Command driveSysIdQuasistatic(SysIdRoutine.Direction direction){
    return driveRoutine.quasistatic(direction);
  }

  public Command driveSysIdDynamic(SysIdRoutine.Direction direction){
    return driveRoutine.dynamic(direction);
  }

  public double getLeft(){
    return leftEncoder.getPosition();
  } 

  public double getRight(){
    return rightEncoder.getPosition();
  } 

  public double getMeters(){
    double leftMeters = (leftEncoder.getPosition() / Constants.factor.kGearRatio) * (factor.wheelDiameter * Math.PI);
    double rightMeters = (rightEncoder.getPosition() / Constants.factor.kGearRatio) * (factor.wheelDiameter * Math.PI);
    
    double avMeters = (leftMeters + rightMeters) / 2;
    return avMeters;
  }

  public double getVelocity(){
    double leftVelocity = ((leftEncoder.getVelocity()  / Constants.factor.kGearRatio) * (factor.wheelDiameter * Math.PI)) / 60;
    double rightVelocity = ((rightEncoder.getVelocity()  / Constants.factor.kGearRatio) * (factor.wheelDiameter * Math.PI)) / 60;

    double averageVelocity = (leftVelocity + rightVelocity) / 2;

    return averageVelocity;
  }

  public double getVolts(){
    double leftVolts = leftLeader.getAppliedOutput() * 12;
    double rightVolts = rightLeader.getAppliedOutput() * 12;

    double avVolts = (leftVolts + rightVolts) / 2;
    return avVolts;
  }
}
