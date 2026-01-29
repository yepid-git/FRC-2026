// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#define PI 3.14159265358979323846

#include "Robot.h"
#include "LimelightHelpers.h"

#include <frc/TimedRobot.h>
#include <frc/filter/SlewRateLimiter.h>
#include <frc/XboxController.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/motorcontrol/PWMSparkMax.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/geometry/Rotation2d.h>
#include <studica/AHRS.h>
#include <frc/SerialPort.h>
#include <frc/SPI.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <rev/SparkMax.h>
#include <frc/motorcontrol/Spark.h>
#include <frc/AnalogEncoder.h>
#include <cameraserver/CameraServer.h>
#include <frc/Timer.h>
#include <units/velocity.h>
#include <frc/BuiltInAccelerometer.h>
#include <frc/PneumaticsModuleType.h>
#include <rev/SparkClosedLoopController.h>
#include <frc/PneumaticHub.h>
#include <frc/DoubleSolenoid.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <cmath>
#include <networktables/NetworkTable.h>

class Robot : public frc::TimedRobot {

  //limelight placeholder variables
  bool foundtarget = false;
  double tx;
  double ty;
  double ta;

  //set location of each wheel relative to center (using WPILib's NWU axes coordinate system)
  frc::Translation2d m_frontLeftLocation{0.213_m, 0.352_m};
  frc::Translation2d m_frontRightLocation{0.213_m, -0.352_m};
  frc::Translation2d m_backLeftLocation{-0.213_m, 0.352_m};
  frc::Translation2d m_backRightLocation{-0.213_m, -0.352_m};


  //sets CAN ID's for the drive wheels
  rev::spark::SparkMax wheelfl{1, rev::spark::SparkLowLevel::MotorType::kBrushless};
  rev::spark::SparkClosedLoopController pidfl = wheelfl.GetClosedLoopController();
  rev::spark::SparkMax wheelfr{5, rev::spark::SparkLowLevel::MotorType::kBrushless};
  rev::spark::SparkClosedLoopController pidfr = wheelfr.GetClosedLoopController();
  rev::spark::SparkMax wheelbl{3, rev::spark::SparkLowLevel::MotorType::kBrushless};
  rev::spark::SparkClosedLoopController pidbl = wheelbl.GetClosedLoopController();
  rev::spark::SparkMax wheelbr{7, rev::spark::SparkLowLevel::MotorType::kBrushless};
  rev::spark::SparkClosedLoopController pidbr = wheelbr.GetClosedLoopController();

  // Configuration objects for PID control of motors via SparkMax's
  rev::spark::SparkBaseConfig driveConfig{};
  rev::spark::SparkBaseConfig steerConfig{};
  rev::spark::SparkBaseConfig otherConfig{};
  rev::spark::SparkBaseConfig hangConfig{};

  // for encoders, consider changing methods to GetAlternateEncoder with AlternateEncoder::Type::kHallEffect or something if you face an error
  // Setting up steering motors using even CAN bus ID's
  // The constructor parameter is the "analog input channel" to use, corresponding to
  // the RoboRIO AnalogIn pins.
  rev::spark::SparkMax rotfl{2, rev::spark::SparkLowLevel::MotorType::kBrushless};
  frc::AnalogEncoder encfl{3};
  rev::spark::SparkMax rotfr{6, rev::spark::SparkLowLevel::MotorType::kBrushless};
  frc::AnalogEncoder encfr{2};
  rev::spark::SparkMax rotbl{4, rev::spark::SparkLowLevel::MotorType::kBrushless};
  frc::AnalogEncoder encbl{0};
  rev::spark::SparkMax rotbr{8, rev::spark::SparkLowLevel::MotorType::kBrushless};
  //dont think this relative encoder is necessary?
  //rev::spark::SparkRelativeEncoder m_brSteerEnc = rotbr.GetEncoder();
  frc::AnalogEncoder encbr{1};


  //shooter CAN ID's
  //Note to self: MIGHT need encoder for rotsh (plan is for continuous alignment with goal)
  rev::spark::SparkMax rotsh{21, rev::spark::SparkLowLevel::MotorType::kBrushless};
  rev::spark::SparkMax firesh{20, rev::spark::SparkLowLevel::MotorType::kBrushless};

  //intake CAN ID
  rev::spark::SparkMax intake{30, rev::spark::SparkLowLevel::MotorType::kBrushless};

  //degrees in int, later converted to degree_t
  int d = 180;
  //speedfactor
  double speedfactor = 2000;
  //timer object
  frc::Timer time;


  //object for roborio's built in accelerometer, returns acceleration on all 3-axes in terms of g-force (1g = 9.8 m/s^2)
  frc::BuiltInAccelerometer acc;

  //AHRS: attitude and heading reference system
  studica::AHRS *ahrs = new studica::AHRS(studica::AHRS::NavXComType::kUSB1);

  //kinematics object
  frc::SwerveDriveKinematics<4> kinematics{
    m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation,
        m_backRightLocation};
  frc::XboxController controller{0};
  //frc::XboxController controller2{1}; maybe use later
  frc::SlewRateLimiter<units::meters_per_second> limitx{9_mps / .5_s};
  frc::SlewRateLimiter<units::meters_per_second> limity{9_mps / .5_s};

  //Controller Mode Variables
  bool m_manual_mode = true;



void RobotInit(){
  //limelight configs
  LimelightHelpers::setPipelineIndex("", 0);
  LimelightHelpers::setLEDMode_ForceOn("");

  //lots of configs
  //Note: TUNE PID's Later (maybe?)
  driveConfig
    .Inverted(true)
    .SetIdleMode(rev::spark::SparkBaseConfig::IdleMode::kBrake);

  //might have to change conversion factors?
  //wheel diameter = 4 inches = 0.1016 meters
  //level unknown, change ratio based on level
  //l3ratio = 6.12;
  //l2ratio = 6.75;
  //l1ratio = 8.14;
  double gearratio = 6.12;

  driveConfig.encoder
    .PositionConversionFactor((PI * 0.1016) / gearratio)
    .VelocityConversionFactor(((PI * 0.1016) / gearratio) / 60.0);

  driveConfig.closedLoop
    .SetFeedbackSensor(rev::spark::FeedbackSensor::kPrimaryEncoder)
    .Pid(0.0001, 0.000001, 0.00000001)
    .IZone(4000);

  steerConfig
    .Inverted(true)
    .SetIdleMode(rev::spark::SparkBaseConfig::IdleMode::kBrake);


  //steer ratio is 150/7:1
  steerConfig.encoder
    .PositionConversionFactor((2.0 * PI) / (150.0 / 7.0))
    .VelocityConversionFactor(((2.0 * PI) / (150.0 / 7.0)) / 60.0);

  steerConfig.closedLoop
    .SetFeedbackSensor(rev::spark::FeedbackSensor::kPrimaryEncoder)
    .Pid(0.0001, 0.000001, 0.00000001)
    .PositionWrappingEnabled(true)
    .IZone(4000);


  //setting configurations for wheel and rotational motors
  wheelfl.Configure(driveConfig, rev::spark::SparkMax::ResetMode::kResetSafeParameters,
    rev::spark::SparkMax::PersistMode::kPersistParameters);
  wheelfr.Configure(driveConfig, rev::spark::SparkMax::ResetMode::kResetSafeParameters,
    rev::spark::SparkMax::PersistMode::kPersistParameters);
  wheelbl.Configure(driveConfig, rev::spark::SparkMax::ResetMode::kResetSafeParameters,
    rev::spark::SparkMax::PersistMode::kPersistParameters);
  wheelbr.Configure(driveConfig, rev::spark::SparkMax::ResetMode::kResetSafeParameters,
    rev::spark::SparkMax::PersistMode::kPersistParameters);

  rotfl.Configure(steerConfig, rev::spark::SparkMax::ResetMode::kResetSafeParameters,
    rev::spark::SparkMax::PersistMode::kPersistParameters);
  rotfr.Configure(steerConfig, rev::spark::SparkMax::ResetMode::kResetSafeParameters,
    rev::spark::SparkMax::PersistMode::kPersistParameters);
  rotbl.Configure(steerConfig, rev::spark::SparkMax::ResetMode::kResetSafeParameters,
    rev::spark::SparkMax::PersistMode::kPersistParameters);
  rotbr.Configure(steerConfig, rev::spark::SparkMax::ResetMode::kResetSafeParameters,
    rev::spark::SparkMax::PersistMode::kPersistParameters); 

  wheelfl.GetEncoder().SetPosition(0);
  wheelfr.GetEncoder().SetPosition(0);
  wheelbl.GetEncoder().SetPosition(0);
  wheelbr.GetEncoder().SetPosition(0);

  rotfl.GetEncoder().SetPosition(encfl.Get() * 2.0 * PI);
  rotfr.GetEncoder().SetPosition(encfr.Get()* 2.0 * PI);
  rotbl.GetEncoder().SetPosition(encbl.Get() * 2.0 * PI);
  rotbr.GetEncoder().SetPosition(encbr.Get()* 2.0 * PI);
  

  //idk what this is for tbh
  //configs something
  otherConfig
    .Inverted(true)
    .SetIdleMode(rev::spark::SparkBaseConfig::IdleMode::kBrake);
  otherConfig.encoder
    .PositionConversionFactor(1)
    .VelocityConversionFactor(1);
  otherConfig.closedLoop
    .SetFeedbackSensor(rev::spark::FeedbackSensor::kPrimaryEncoder)
    .Pid(0.0275, 0.000002, 0.00000001)
    .IZone(4000);

  //configurations for hang motors
  hangConfig
    .Inverted(true)
    .SetIdleMode(rev::spark::SparkBaseConfig::IdleMode::kBrake);
  hangConfig.encoder
    .PositionConversionFactor(1)
    .VelocityConversionFactor(1);
  hangConfig.closedLoop
    .SetFeedbackSensor(rev::spark::FeedbackSensor::kPrimaryEncoder)
    .Pid(0.0005, 0.0000001, 0.0)
    .IZone(4000);


  ahrs->Reset();
  ahrs->ResetDisplacement();
  ahrs->SetAngleAdjustment(0);
}



void RobotPeriodic() {

  foundtarget = LimelightHelpers::getTV("");
  tx = LimelightHelpers::getTX("");
  ty = LimelightHelpers::getTY("");
  ta = LimelightHelpers::getTA("");

  frc::SmartDashboard::PutBoolean("HasTarget:", foundtarget);
  frc::SmartDashboard::PutNumber("TX", tx);
  frc::SmartDashboard::PutNumber("TY", ty);
  frc::SmartDashboard::PutNumber("TA", ta);

}


void AutonomousInit() {
  time.Reset();
  time.Start();
}

void AutonomousPeriodic() {

}

void TeleopInit() {
  time.Stop();
  time.Reset();
  speedfactor = 2000;
}

void TeleopPeriodic() {
  //x, y, turn
  //for now, just calling drive on it's own
  Drive(-controller.GetLeftY(), -controller.GetLeftX(), -controller.GetRightX());
}

void SetState(frc::SwerveModuleState optState, rev::spark::SparkMax& driveSpark, rev::spark::SparkMax& steerSpark){
  double angleThr{2*PI/360};
  double deltaAngle = optState.angle.Radians().value() - steerSpark.GetEncoder().GetPosition();
  if ((fabs(optState.speed.value()) < 0.001) && (fabs(deltaAngle) < angleThr)) {
    driveSpark.Set(0);
  } else {
    driveSpark.GetClosedLoopController().SetReference(optState.speed.value(), rev::spark::SparkBase::ControlType::kVelocity);
    steerSpark.GetClosedLoopController().SetReference(optState.angle.Radians().value(), rev::spark::SparkBase::ControlType::kPosition);
  }
}

void Drive(double x, double y, double rotate){
  //grabs robot's angle relative to driver station, in other words it's current field orientation
  d = 360 - ahrs->GetAngle();

  if (std::abs(x) < 0.1) x = 0;
  if (std::abs(y) < 0.1) y = 0;
  if (std::abs(rotate) < 0.1) rotate = 0;

  units::degree_t degr{d};
  frc::Rotation2d rot2d{degr}; //rot2d reflects the AHRS gyroscope orientation

  frc::SmartDashboard::PutNumber("Drive:x", x);
  frc::SmartDashboard::PutNumber("Drive:y", y);
  frc::SmartDashboard::PutNumber("Drive:rotate", rotate);
  frc::SmartDashboard::PutNumber("Drive: AHRS (rot2d)", rot2d.Degrees().value());
  frc::SmartDashboard::PutNumber("encfl.Get", encfl.Get());
  frc::SmartDashboard::PutNumber("encfr.Get", encfr.Get());
  frc::SmartDashboard::PutNumber("encbl.Get", encbl.Get());
  frc::SmartDashboard::PutNumber("encbr.Get", encbr.Get());


  units::radians_per_second_t rad{rotate*8};
  units::meters_per_second_t speedy{y*4};
  units::meters_per_second_t speedx{x*4};
  //uses the slewrate limiter to determine necessary chassis speeds
  frc::ChassisSpeeds speeds = frc::ChassisSpeeds::FromFieldRelativeSpeeds(
    limitx.Calculate(speedx),
    limity.Calculate(speedy),
    rad * 1.2,  // sensitivity multiplier?? increase if rotation is sluggish, decrease if jittery
    rot2d  // This is what enables field-oriented control
  );

  //converts the speeds to swerve module states
  auto [fl, fr, bl, br] = kinematics.ToSwerveModuleStates(speeds);


  //experiment: getting wpilib to optimize angles instead (eliminating need for complex swerve math)
  frc::Rotation2d flAngle{units::radian_t{rotfl.GetEncoder().GetPosition()}};
  frc::Rotation2d frAngle{units::radian_t{rotfr.GetEncoder().GetPosition()}};
  frc::Rotation2d blAngle{units::radian_t{rotbl.GetEncoder().GetPosition()}};
  frc::Rotation2d brAngle{units::radian_t{rotbr.GetEncoder().GetPosition()}};


  auto flOptimized = frc::SwerveModuleState::Optimize(fl, flAngle);
  auto frOptimized = frc::SwerveModuleState::Optimize(fr, frAngle);
  auto blOptimized = frc::SwerveModuleState::Optimize(bl, blAngle);
  auto brOptimized = frc::SwerveModuleState::Optimize(br, brAngle);

  /*
  fl.Optimize(flAngle);
  fr.Optimize(frAngle);
  bl.Optimize(blAngle);
  br.Optimize(brAngle);
  */

  SetState(flOptimized, wheelfl, rotfl);
  SetState(frOptimized, wheelfr, rotfr);
  SetState(blOptimized, wheelbl, rotbl);
  SetState(brOptimized, wheelbr, rotbr);

}

/*
void DisabledInit() {}

void DisabledPeriodic() {}

void TestInit() {}

void TestPeriodic() {}

void SimulationInit() {}

void SimulationPeriodic() {}
*/

};

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
