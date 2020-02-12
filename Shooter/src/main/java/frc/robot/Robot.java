/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  public CANSparkMax primary = new CANSparkMax(9, MotorType.kBrushless);
  public CANSparkMax secondary = new CANSparkMax(10, MotorType.kBrushless);

  public WPI_TalonSRX feeder_1 = new WPI_TalonSRX(13); 
  public WPI_TalonSRX feeder_2 = new WPI_TalonSRX(20);
  
  private static final int deviceID = 1;
  private CANPIDController pidController;
  private CANEncoder encoder;
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM;

  public XboxController controller = new XboxController(0);

  private PIDController shooterPid;
  private PIDController indexPid;

  @Override
  public void robotInit() {
    primary.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 20);
    secondary.follow(primary);
    //secondary.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 20);

    pidController = primary.getPIDController();
    encoder = primary.getEncoder();

    // PID coefficients
    kP = 0.0001; 
    kI = 0;
    kD = 0; 
    kIz = 0; 
    kFF = 0.000015; 
    kMaxOutput = 1; 
    kMinOutput = -1;
    maxRPM = 5700;

    // set PID coefficients
    pidController.setP(kP);
    pidController.setI(kI);
    pidController.setD(kD);
    pidController.setIZone(kIz);
    pidController.setFF(kFF);
    pidController.setOutputRange(kMinOutput, kMaxOutput);

    // display PID coefficients on SmartDashboard
    SmartDashboard.putNumber("P Gain", kP);
    SmartDashboard.putNumber("I Gain", kI);
    SmartDashboard.putNumber("D Gain", kD);
    SmartDashboard.putNumber("I Zone", kIz);
    SmartDashboard.putNumber("Feed Forward", kFF);
    SmartDashboard.putNumber("Max Output", kMaxOutput);
    SmartDashboard.putNumber("Min Output", kMinOutput);
    SmartDashboard.putNumber("IMaxAccum", 0);

    // shooterPid = new PIDController(SmartDashboard.getNumber("P", 0),
    //                                SmartDashboard.getNumber("I", 0),
    //                                SmartDashboard.getNumber("D", 0));

    
    // shooterPid.setTolerance(0, 0);
    // indexPid = new PIDController(SmartDashboard.getNumber("P_index", 0),
    //                              SmartDashboard.getNumber("I_index", 0),
    //                              SmartDashboard.getNumber("D_index", 0));

    primary.restoreFactoryDefaults();
    secondary.restoreFactoryDefaults();

    primary.setInverted(false);
    secondary.setInverted(true);

    primary.setIdleMode(IdleMode.kCoast);
    secondary.setIdleMode(IdleMode.kCoast);

    feeder_1.setInverted(true);
    feeder_2.setInverted(true);

    // SmartDashboard.putNumber("Target RPM", 0);
    //shooterPid.setSetpoint(SmartDashboard.getNumber("Target RPM", 4824.6));
    
    // SmartDashboard.putNumber("Target Index RPM", 0);
    //indexPid.setSetpoint(SmartDashboard.getNumber("Target Index RPM", 10));

    // feeder_1.getSensorCollection().setQuadraturePosition(0,0);
  }

  
  @Override
  public void robotPeriodic() {}

  public static double boundValue(double value) {
    if (value > 1) {
      return 1;
    }
    if (value < -1) {
      return -1;
    }
    return value;
  }

  @Override
  public void autonomousInit() {}

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopPeriodic() {
    // SmartDashboard.putNumber("indexer rpm", feeder_1.getSelectedSensorVelocity());
    // SmartDashboard.putNumber("indexer pos", feeder_1.getSelectedSensorPosition());
    // SmartDashboard.putNumber("sensor quadrature position?", feeder_1.getSensorCollection().getQuadraturePosition());

    // read PID coefficients from SmartDashboard
    double p = SmartDashboard.getNumber("P Gain", 0);
    double i = SmartDashboard.getNumber("I Gain", 0);
    double d = SmartDashboard.getNumber("D Gain", 0);
    double iz = SmartDashboard.getNumber("I Zone", 0);
    double ff = SmartDashboard.getNumber("Feed Forward", 0);
    double max = SmartDashboard.getNumber("Max Output", 0);
    double min = SmartDashboard.getNumber("Min Output", 0);

   if(i == 0) {
     pidController.setIAccum(0);
   }
    // if PID coefficients on SmartDashboard have changed, write new values to controller
    if((p != kP)) { pidController.setP(p); kP = p; }
    if((i != kI)) { pidController.setI(i); kI = i; }
    if((d != kD)) { pidController.setD(d); kD = d; }
    if((iz != kIz)) { pidController.setIZone(iz); kIz = iz; }
    if((ff != kFF)) { pidController.setFF(ff); kFF = ff; }
    if((max != kMaxOutput) || (min != kMinOutput)) { 
      pidController.setOutputRange(min, max); 
      kMinOutput = min; kMaxOutput = max; 
    }

    /**
     * PIDController objects are commanded to a set point using the 
     * SetReference() method.
     * 
     * The first parameter is the value of the set point, whose units vary
     * depending on the control type set in the second parameter.
     * 
     * The second parameter is the control type can be set to one of four 
     * parameters:
     *  com.revrobotics.ControlType.kDutyCycle
     *  com.revrobotics.ControlType.kPosition
     *  com.revrobotics.ControlType.kVelocity
     *  com.revrobotics.ControlType.kVoltage
     */
    // double setPoint = controller.getY() * maxRPM;
    double setPoint = SmartDashboard.getNumber("SetPoint", 0);
    pidController.setReference(setPoint, ControlType.kVelocity);
    
    SmartDashboard.putNumber("SetPoint", setPoint);
    SmartDashboard.putNumber("ProcessVariable", encoder.getVelocity());

    pidController.setIMaxAccum(SmartDashboard.getNumber("IMaxAccum", 0), 0);

    // shooterPid.setSetpoint(SmartDashboard.getNumber("Target RPM", 0));
    //shooterPid.setSetpoint(4000);
    SmartDashboard.putNumber("Left speed", primary.getEncoder().getVelocity());
    SmartDashboard.putNumber("Right speed", secondary.getEncoder().getVelocity());
    
    SmartDashboard.putNumber("I Accumulator", pidController.getIAccum());

    // double nextOutput = shooterPid.calculate(primary.getEncoder().getVelocity(),
    //                                          SmartDashboard.getNumber("Target RPM", 4824.6));
    // double preBoundValue = nextOutput / 10000;
    // double output = shooterPid.calculate(primary.getEncoder().getVelocity());
    // SmartDashboard.putNumber("nextOutput", nextOutput);
    // SmartDashboard.putNumber("preBoundValue", preBoundValue);
    // double shooterPidOutput = boundValue(output / 10000);
    // shooterPid.setSetpoint(SmartDashboard.getNumber("Target RPM", 4824.6));
    //primary.set(shooterPidOutput);
    //secondary.set(shooterPidOutput);
    SmartDashboard.putNumber("Counts per revolution", primary.getEncoder().getCountsPerRevolution());
    SmartDashboard.putNumber("Encoder Position", primary.getEncoder().getPosition());

    // SmartDashboard.putNumber("PID output", shooterPidOutput);
    //System.out.println(shooterPidOutput);
    // SmartDashboard.putNumber("PID error", shooterPid.getVelocityError());
    
    // SmartDashboard.putBoolean("PID at Target", shooterPid.atSetpoint());

    // indexPid.setPID(SmartDashboard.getNumber("P_index", 0),
    //            SmartDashboard.getNumber("I_index", 0),
    //            SmartDashboard.getNumber("D_index", 0));

     if(controller.getBumper(Hand.kLeft)) {
      // double indexPidOutput = boundValue(indexPid.calculate(feeder_1.getSensorCollection().getQuadratureVelocity(),
      //   SmartDashboard.getNumber("Target Index RPM", 5000)));

      // feeder_1.set(indexPidOutput);
      feeder_1.set(0.75);
    }
    if(controller.getBumper(Hand.kRight)) {
      feeder_2.set(-0.5);
    }
    if(!controller.getBumper(Hand.kLeft)) {
      feeder_1.set(0);
    }
    if(!controller.getBumper(Hand.kRight)) {
      feeder_2.set(0);
    }
  }

  @Override
  public void testPeriodic() {}
}
