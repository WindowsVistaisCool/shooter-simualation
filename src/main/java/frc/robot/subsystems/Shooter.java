// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXSimCollection;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {

  private TalonFX flywheel;
  private TalonFXSimCollection flywheelSimCollection;

  private FlywheelSim flywheelSim;

  /** Creates a new Shooter. */
  public Shooter() {
    flywheel = new TalonFX(0);
    flywheelSimCollection = flywheel.getSimCollection();

    flywheel.setNeutralMode(NeutralMode.Coast);
    flywheel.configClosedloopRamp(0.02);

    flywheelSim = new FlywheelSim(
      DCMotor.getFalcon500(1),
      1,
      Units.inchesToMeters(10)
    );

    CommandScheduler.getInstance().registerSubsystem(this);
  }

  @Override
  public void periodic() {
    flywheelSimCollection.setBusVoltage(RobotController.getBatteryVoltage());
    flywheelSim.setInput(flywheelSimCollection.getMotorOutputLeadVoltage());
    flywheelSim.update(0.02);

    RoboRioSim.setVInVoltage(
      BatterySim.calculateDefaultBatteryLoadedVoltage(flywheelSim.getCurrentDrawAmps())
    );
  }

  public void setPower(double power) {
    flywheel.set(TalonFXControlMode.PercentOutput, power);
  }

  public void stop() {
    setPower(0);
  }
}
