#define PI 3.14159265358979323846

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
  // These analog encoders, when .Get() is used, provide the angle in ROTATIONS (0 to 1.0) 
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

  //degrees in double, later converted to degree_t
  double d = 180;
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

  //Slew rate limiter is REALLY high, 18 m/s^2 is the limit on acceleration (basically no limit)
  //change if robot shoots forwards too fast!
  frc::SlewRateLimiter<units::meters_per_second> limitx{9_mps / .5_s};
  frc::SlewRateLimiter<units::meters_per_second> limity{9_mps / .5_s};

  //Controller Mode Variables
  bool m_manual_mode = true;



void RobotInit(){
  //limelight configs (disabled for now)
  /*
  LimelightHelpers::setPipelineIndex("", 0);
  LimelightHelpers::setLEDMode_ForceOn("");
  */

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
  double gearratio = 8.14;


  //The conversion factor here turns rotations, to meters. In one rotation, the wheel travels
  //diameter * PI / gear ratio
  driveConfig.encoder
    .PositionConversionFactor((PI * 0.1016) / gearratio)
    .VelocityConversionFactor(((PI * 0.1016) / gearratio) / 60.0);

  driveConfig.closedLoop
    .SetFeedbackSensor(rev::spark::FeedbackSensor::kPrimaryEncoder)
    //pid might be too small?
    .Pid(0.1, 0.000001, 0.00000001)
    .IZone(4000);

  steerConfig
    .Inverted(true)
    .SetIdleMode(rev::spark::SparkBaseConfig::IdleMode::kBrake);


  //steer gear ratio is 150/7:1
  //one rotation is 2pi radians
  //divide that by the steering gear ratio gets the amount of radians traveled per rotation
  steerConfig.encoder
    .PositionConversionFactor((2.0 * PI) / (150.0 / 7.0))
    .VelocityConversionFactor(((2.0 * PI) / (150.0 / 7.0)) / 60.0);


  //for now, disable position wrapping
  steerConfig.closedLoop
    .SetFeedbackSensor(rev::spark::FeedbackSensor::kPrimaryEncoder)
    .Pid(0.3, 0.000001, 0.00000001)
    .PositionWrappingEnabled(true)
    .PositionWrappingInputRange(-PI, PI)
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


  //rotation motors need seeding to know what angle they start at
  //similar to the absolute encoders, the analog encoders also return the wheel's angle in ROTATIONS
  //we want RADIANS!!!
  //2pi rad per rotation
  //note to self: apparently we need offsets, fix later.

  double floff = 0.0;
  double froff = 0.0;
  double bloff = 0.0;
  double broff = 0.0;
  
  rotfl.GetEncoder().SetPosition((encfl.Get() + floff) * 2.0 * PI);
  rotfr.GetEncoder().SetPosition((encfr.Get() + froff) * 2.0 * PI);
  rotbl.GetEncoder().SetPosition((encbl.Get() + bloff) * 2.0 * PI);
  rotbr.GetEncoder().SetPosition((encbr.Get() + broff) * 2.0 * PI);
  

  //idk what this is for tbh
  //unused for now
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

  //every 20ms, robot receives new data from limelight
  //currently not important for purposes of drive testing, but it will be in auto
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


//every 20ms during teleop robot reads the joystick's percentages
void TeleopPeriodic() {
  //x, y, turn
  //for now, just calling drive on it's own
  //joysticks are inverted because wpi NWU axes co-ordinate system is weird, search it up if interested
  Drive(-controller.GetLeftY(), -controller.GetLeftX(), -controller.GetRightX());
}

//SetState takes in optimal state, and both drive and steer spark motor controller objects
void SetState(frc::SwerveModuleState optState, rev::spark::SparkMax& driveSpark, rev::spark::SparkMax& steerSpark){

  /* The driver sparks PID controller gets the speed value from optState, and then does 
  its own internal calculations to achieve that value.
  Basically the spark looks at it's own encoder to get it's current velocity,
  then does it's own internal math to calculate the voltage necessary to reach
  and maintain the target velocity. 
  If the motor spins too fast, it decreases voltage.
  If the motor spins too slow, it increases voltage.
  */
  driveSpark.GetClosedLoopController().SetReference(
    optState.speed.value(), 
    rev::spark::SparkBase::ControlType::kVelocity
  );

  //same with steer sparks PID controller
  steerSpark.GetClosedLoopController().SetReference(
    optState.angle.Radians().value(), 
    rev::spark::SparkBase::ControlType::kPosition
  );
}


//drive grabs the joystick PERCENTAGES, and converts them into VELOCITY
void Drive(double x, double y, double rotate){
  //grabs robot's angle relative to driver station, in other words it's current field orientation
  d = 360 - ahrs->GetAngle();

  /* 
  since joysticks don't truly return to "zero", the deadbands (if statements) are there to ignore
  input IF the joystick values are negligible
  */

  if (std::abs(x) < 0.1) x = 0;
  if (std::abs(y) < 0.1) y = 0;
  if (std::abs(rotate) < 0.1) rotate = 0;

  //if the controllers have no input, just sets all the motors speeds to 0
  if (x == 0 && y == 0 && rotate == 0) {
    wheelfl.Set(0); wheelfr.Set(0); wheelbl.Set(0); wheelbr.Set(0);
    rotfl.Set(0);   rotfr.Set(0);   rotbl.Set(0);   rotbr.Set(0);
    return; 
  }

  //rot2d reflects the AHRS gyroscope orientation
  frc::Rotation2d rot2d = frc::Rotation2d(units::degree_t{-ahrs->GetAngle()});


  //bunch of debugging utilities
  frc::SmartDashboard::PutNumber("Drive:x", x);
  frc::SmartDashboard::PutNumber("Drive:y", y);
  frc::SmartDashboard::PutNumber("Drive:rotate", rotate);
  frc::SmartDashboard::PutNumber("Drive: AHRS (rot2d)", rot2d.Degrees().value());
  frc::SmartDashboard::PutNumber("encfl.Get", encfl.Get());
  frc::SmartDashboard::PutNumber("encfr.Get", encfr.Get());
  frc::SmartDashboard::PutNumber("encbl.Get", encbl.Get());
  frc::SmartDashboard::PutNumber("encbr.Get", encbr.Get());



  //this is where joystick percentages become velocity!
  //max speeds become 1 x factor units / sec
  //max x & y speed become 4m/s
  //max rotation speed is 3 rad/s
  units::radians_per_second_t rad{rotate*3};
  units::meters_per_second_t speedy{y*4};
  units::meters_per_second_t speedx{x*4};

  /* ChassisSpeeds::FromFieldRelativeSpeeds takes in desired x, desired y, and angular velocities
  as well as the robots current angle
  */
  frc::ChassisSpeeds speeds = frc::ChassisSpeeds::FromFieldRelativeSpeeds(
    //uses the slewrate limiter to determine necessary chassis speeds
    limitx.Calculate(speedx),
    limity.Calculate(speedy),
    rad * 1.2,  // sensitivity multiplier?? increase if rotation is sluggish, decrease if jittery
    rot2d  // This is what enables field-oriented control
  );

  //converts the speeds to swerve module states

  auto modules = kinematics.ToSwerveModuleStates(speeds);

  //safety to prevent wheels from spinning too fast
  kinematics.DesaturateWheelSpeeds(&modules, 4_mps);

  //just stores the swerve module states in each motor
  auto [fl, fr, bl, br] = modules;


  //experiment: getting wpilib to optimize angles instead (eliminating need for complex swerve math)
  frc::Rotation2d flAngle{units::radian_t{rotfl.GetEncoder().GetPosition()}};
  frc::Rotation2d frAngle{units::radian_t{rotfr.GetEncoder().GetPosition()}};
  frc::Rotation2d blAngle{units::radian_t{rotbl.GetEncoder().GetPosition()}};
  frc::Rotation2d brAngle{units::radian_t{rotbr.GetEncoder().GetPosition()}};


  //SwerveModuleState::Optimize takes in the state, and the ideal angle, then optimizes the state to include that angle
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

  //set state takes in the perfectly optimized state, the movement controller, and steering controller
  //optimized state contains two values, the speed and angle

  SetState(flOptimized, wheelfl, rotfl);
  SetState(frOptimized, wheelfr, rotfr);
  SetState(blOptimized, wheelbl, rotbl);
  SetState(brOptimized, wheelbr, rotbr);

}


void DisabledInit() {}

void DisabledPeriodic() {}

void TestInit() {}

void TestPeriodic() {
  frc::SmartDashboard::PutNumber("encfl.Get", encfl.Get());
  frc::SmartDashboard::PutNumber("encfr.Get", encfr.Get());
  frc::SmartDashboard::PutNumber("encbl.Get", encbl.Get());
  frc::SmartDashboard::PutNumber("encbr.Get", encbr.Get());
}

void SimulationInit() {}

void SimulationPeriodic() {}


};

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
