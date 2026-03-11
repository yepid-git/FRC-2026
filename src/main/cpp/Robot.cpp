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
#include <rev/SparkFlex.h>
#include <frc/MathUtil.h>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <units/time.h>
#include <frc/estimator/SwerveDrivePoseEstimator.h>

//includes for auto
#include <pathplanner/lib/auto/AutoBuilder.h>
#include <pathplanner/lib/config/RobotConfig.h>
#include <pathplanner/lib/controllers/PPHolonomicDriveController.h>
#include <frc2/command/CommandScheduler.h>
#include <frc2/command/Commands.h>
#include <frc/DriverStation.h>
#include <pathplanner/lib/util/DriveFeedforwards.h>
#include <pathplanner/lib/commands/PathPlannerAuto.h>
#include <pathplanner/lib/path/PathPlannerPath.h>



class Robot : public frc::TimedRobot {

  //limelight placeholder variables
  bool hastarget = false;
  double tx;
  double ty;
  double ta;

  //vertical and horizontal turret
  double VerticalSpeed = 0.1; 
  double HorizontalSpeed = 0.2;
  double IndexerSpeed = 1;
  double HopperSpeed = 0.7; //make negative if too fast
  double HangSpeed = 0.5;

  char color = 'b'; //color variable 

  //member for the auto command
  frc2::CommandPtr autoCommand = frc2::cmd::None();

  //placeholder variable for the goal hub's position on the field
  frc::Translation2d GoalPosition{4.612_m, -4.021_m};


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
  rev::spark::SparkBaseConfig shooterConfig{};

  // for encoders, consider changing methods to GetAlternateEncoder with AlternateEncoder::Type::kHallEffect or something if you face an error
  // Setting up steering motors using even CAN bus ID's
  // The constructor parameter is the "analog input channel" to use, corresponding to
  // the RoboRIO AnalogIn pins.
  // These analog encoders, when .Get() is used, provide the angle in ROTATIONS (0 to 1.0) 
  //additionally, initializes a closed loop controller for each rotational motor for Xstop to function

  rev::spark::SparkMax rotfl{2, rev::spark::SparkLowLevel::MotorType::kBrushless};
  frc::AnalogEncoder encfl{3};

  rev::spark::SparkMax rotfr{6, rev::spark::SparkLowLevel::MotorType::kBrushless};
  frc::AnalogEncoder encfr{2};


  rev::spark::SparkMax rotbl{4, rev::spark::SparkLowLevel::MotorType::kBrushless};
  frc::AnalogEncoder encbl{0};


  rev::spark::SparkMax rotbr{8, rev::spark::SparkLowLevel::MotorType::kBrushless};
  frc::AnalogEncoder encbr{1};



  //shooter CAN ID's  
  rev::spark::SparkFlex firesh{20, rev::spark::SparkLowLevel::MotorType::kBrushless}; //Leader motor
  rev::spark::SparkFlex firesh2{21, rev::spark::SparkLowLevel::MotorType::kBrushless}; //Follower motor

  rev::spark::SparkClosedLoopController pidfiresh = firesh.GetClosedLoopController();
  rev::spark::SparkClosedLoopController pidfiresh2 = firesh2.GetClosedLoopController();

  //configs for shooters
  rev::spark::SparkBaseConfig shooterLeaderConfig{};
  rev::spark::SparkBaseConfig shooterFollowerConfig{};


  //shooter rotation ID's
  //each motor rotates shooter in x or y axis, ignore vertical for now
  rev::spark::SparkMax HorizontalTurret{22, rev::spark::SparkLowLevel::MotorType::kBrushless};
  rev::spark::SparkMax VerticalTurret{23, rev::spark::SparkLowLevel::MotorType::kBrushless};


  rev::spark::SparkBaseConfig HorizontalTurretConfig{};
  rev::spark::SparkBaseConfig VerticalTurretConfig{};

  //indexer CAN ID's
  rev::spark::SparkMax Indexer{10, rev::spark::SparkLowLevel::MotorType::kBrushless};
  rev::spark::SparkBaseConfig IndexerConfig{};


  rev::spark::SparkMax Intake{11, rev::spark::SparkLowLevel::MotorType::kBrushless};
  rev::spark::SparkBaseConfig IntakeConfig{};

  //hang CAN ID
  rev::spark::SparkMax Hang{30, rev::spark::SparkLowLevel::MotorType::kBrushless};
  rev::spark::SparkBaseConfig HangConfig{};

  //spindexer/hopper
  rev::spark::SparkFlex Hopper{31, rev::spark::SparkLowLevel::MotorType::kBrushless};
  rev::spark::SparkBaseConfig hopperConfig{};
  
  frc::Timer time;


  //object for roborio's built in accelerometer, returns acceleration on all 3-axes in terms of g-force (1g = 9.8 m/s^2)
  frc::BuiltInAccelerometer acc;

  //AHRS: attitude and heading reference system
  studica::AHRS *ahrs = new studica::AHRS(studica::AHRS::NavXComType::kMXP_SPI);

  //kinematics object
  frc::SwerveDriveKinematics<4> kinematics{
    m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation,
        m_backRightLocation};
  frc::XboxController controller{0};
  //frc::XboxController controller2{1}; maybe use later

  //limits acceleration to 6m/s^2
  frc::SlewRateLimiter<units::meters_per_second> limitx{3_mps / .5_s};
  frc::SlewRateLimiter<units::meters_per_second> limity{3_mps / .5_s};
  frc::SlewRateLimiter<units::radians_per_second> limitrot{3_rad_per_s / .5_s};

  //Controller Mode Variables
  bool m_manual_mode = true;

  //declaring odometry object at class level using pointer, to avoid scoping issues
  //std::unique_ptr<frc::SwerveDriveOdometry<4>> odometry; swapped for pose estimator!
  std::unique_ptr<frc::SwerveDrivePoseEstimator<4>> poseEstimator;
  frc::Pose2d pose;


void RobotInit(){
  //color change
  auto alliance = frc::DriverStation::GetAlliance();
  if (alliance && alliance.value() == frc::DriverStation::Alliance::kRed) {
    color = 'r';
  }


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
    .VelocityFF(0.25)
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

  steerConfig.closedLoop
    .SetFeedbackSensor(rev::spark::FeedbackSensor::kPrimaryEncoder)
    .Pid(0.4, 0, 0.02)
    .PositionWrappingEnabled(true)
    .PositionWrappingInputRange(-PI, PI)
    .IZone(0.1);

  //leader shooter config 
  shooterLeaderConfig
    .Inverted(true)
    .SetIdleMode(rev::spark::SparkBaseConfig::IdleMode::kCoast);
  
  shooterLeaderConfig.closedLoop
    .SetFeedbackSensor(rev::spark::FeedbackSensor::kPrimaryEncoder)
    .Pid(0.0001, 0.0, 0.0)
    //decrease if motor fires at full power
    .VelocityFF(0.000147)
    .IZone(0);

  //sets the follower shooter to actually follow the leader
  shooterFollowerConfig
  .SetIdleMode(rev::spark::SparkBaseConfig::IdleMode::kCoast);
  //.Follow(firesh, true);

  shooterFollowerConfig.closedLoop
    .SetFeedbackSensor(rev::spark::FeedbackSensor::kPrimaryEncoder)
    .Pid(0.0001, 0.0, 0.0)
    //decrease if motor fires at full power
    .VelocityFF(0.000147)
    .IZone(0);

  hopperConfig
    .Inverted(true)
    .SetIdleMode(rev::spark::SparkBaseConfig::IdleMode::kCoast);
  
  hopperConfig.closedLoop
    .SetFeedbackSensor(rev::spark::FeedbackSensor::kPrimaryEncoder)
    .Pid(0.01, 0.0, 0.0)
    //decrease if motor fires at full power
    .VelocityFF(0.000147)
    .IZone(0);

  HorizontalTurretConfig
  .Inverted(false)
  .SetIdleMode(rev::spark::SparkBaseConfig::IdleMode::kBrake);

  //configs for the turret's rotational motors
  HorizontalTurretConfig.encoder
    .PositionConversionFactor((2.0 * PI) / 50) // 50:1 Gearbox ratio
    .VelocityConversionFactor(((2.0 * PI) / 50 / 60.0));

  HorizontalTurretConfig.closedLoop
  .SetFeedbackSensor(rev::spark::FeedbackSensor::kPrimaryEncoder)
  .Pid(0.2, 0.0, 0.0)
  .PositionWrappingEnabled(false) //experiment
  .PositionWrappingInputRange(-PI, PI)
  .OutputRange(-0.4, 0.4);

  HorizontalTurretConfig.softLimit
    .ForwardSoftLimit(PI)  // 180 degrees
    .ForwardSoftLimitEnabled(true)
    .ReverseSoftLimit(-PI) // -180 degrees
    .ReverseSoftLimitEnabled(true);


  VerticalTurretConfig
    .Inverted(true)
    .SetIdleMode(rev::spark::SparkBaseConfig::IdleMode::kBrake);
  
  VerticalTurretConfig.closedLoop
    .SetFeedbackSensor(rev::spark::FeedbackSensor::kPrimaryEncoder)
    .Pid(0.2, 0.0, 0.0)
    .PositionWrappingEnabled(false)
    .OutputRange(-0.1, 0.1);

  VerticalTurretConfig.encoder
    .PositionConversionFactor((2.0 * PI) / 36 * 4) // 36:1 Gearbox ratio
    .VelocityConversionFactor(((2.0 * PI) / 36 * 4) / 60.0);

  
  IndexerConfig.closedLoop
  .SetFeedbackSensor(rev::spark::FeedbackSensor::kPrimaryEncoder)
  .Pid(0.3, 0.0, 0.0)
  .PositionWrappingEnabled(false);

  IntakeConfig.closedLoop
  .SetFeedbackSensor(rev::spark::FeedbackSensor::kPrimaryEncoder)
  .Pid(0.3, 0.0, 0.0)
  .PositionWrappingEnabled(false)
  .OutputRange(-0.5, 0.5);

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



  //shooter configs
  firesh.Configure(shooterLeaderConfig, rev::spark::SparkBase::ResetMode::kResetSafeParameters,
    rev::spark::SparkBase::PersistMode::kPersistParameters);

  firesh2.Configure(shooterFollowerConfig, rev::spark::SparkBase::ResetMode::kResetSafeParameters,
    rev::spark::SparkBase::PersistMode::kPersistParameters);

  HorizontalTurret.Configure(HorizontalTurretConfig, rev::spark::SparkBase::ResetMode::kResetSafeParameters,
    rev::spark::SparkBase::PersistMode::kPersistParameters);
  
  VerticalTurret.Configure(VerticalTurretConfig, rev::spark::SparkBase::ResetMode::kResetSafeParameters,
    rev::spark::SparkBase::PersistMode::kPersistParameters);

  Indexer.Configure(IndexerConfig, rev::spark::SparkBase::ResetMode::kResetSafeParameters,
    rev::spark::SparkBase::PersistMode::kPersistParameters);

  Intake.Configure(IntakeConfig,rev::spark::SparkBase::ResetMode::kResetSafeParameters,
    rev::spark::SparkBase::PersistMode::kPersistParameters);
  
  Hang.Configure(HangConfig, rev::spark::SparkBase::ResetMode::kResetSafeParameters,
    rev::spark::SparkBase::PersistMode::kPersistParameters);

  Hopper.Configure(hopperConfig,rev::spark::SparkBase::ResetMode::kResetSafeParameters,
    rev::spark::SparkBase::PersistMode::kPersistParameters);
  
    //rotation motors need seeding to know what angle they start at
  //similar to the absolute encoders, the analog encoders also return the wheel's angle in ROTATIONS
  //we want RADIANS!!!
  //2pi rad per rotation

  //when finding offset values, if offset is less than or equal to 0.5, keep it positive as is
  //if offset is negative, set it equal to -(1-offset) 
  double floff = 0.5;
  double froff = 0.14;
  double bloff = -0.1;
  double broff = 0;


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
  HangConfig
    .Inverted(true)
    .SetIdleMode(rev::spark::SparkBaseConfig::IdleMode::kBrake);
  HangConfig.encoder
    .PositionConversionFactor(1)
    .VelocityConversionFactor(1);
  HangConfig.closedLoop
    .SetFeedbackSensor(rev::spark::FeedbackSensor::kPrimaryEncoder)
    .Pid(0.0005, 0.0000001, 0.0)
    .IZone(4000);

    
  
  //odometry object
  //tracks robot position on field by using the motor encoders
  /* removed because incompatible with limelight
  odometry = std::make_unique<frc::SwerveDriveOdometry<4>>(
    kinematics,
    frc::Rotation2d{units::degree_t{ahrs->GetYaw()}},
    GetSwervePositions(),
    frc::Pose2d{0_m, 0_m, 0_rad}
  );
  pose = odometry->GetPose();
  */

  poseEstimator = std::make_unique<frc::SwerveDrivePoseEstimator<4>>(
    kinematics,
    frc::Rotation2d{units::degree_t{ahrs->GetAngle()}},
    GetSwervePositions(),
    frc::Pose2d{0_m, 0_m, 0_rad}
  );

  /*Lambda function used to gain the
  robot-relative speeds from the encoders
  Pathplanner needs this to know how fast the robot is moving
  */
  auto getRobotRelativeSpeeds = [this](){
    return kinematics.ToChassisSpeeds(
      frc::SwerveModuleState{
        units::meters_per_second_t{wheelfl.GetEncoder().GetVelocity()},
        frc::Rotation2d{units::radian_t{rotfl.GetEncoder().GetPosition()}}
      },
      frc::SwerveModuleState{
        units::meters_per_second_t{wheelfr.GetEncoder().GetVelocity()},
        frc::Rotation2d{units::radian_t{rotfr.GetEncoder().GetPosition()}}
      },
      frc::SwerveModuleState{
        units::meters_per_second_t{wheelbl.GetEncoder().GetVelocity()},
        frc::Rotation2d{units::radian_t{rotbl.GetEncoder().GetPosition()}}
      },
      frc::SwerveModuleState{
        units::meters_per_second_t{wheelbr.GetEncoder().GetVelocity()},
        frc::Rotation2d{units::radian_t{rotbr.GetEncoder().GetPosition()}}
      }
    );
  };


  //pathplanner gives robot relative speeds
  //we apply them

  //notice how this is VERY similar to regular drive code
  auto driveRobotRelative = [this](frc::ChassisSpeeds speeds, pathplanner::DriveFeedforwards ff) {
    speeds = frc::ChassisSpeeds::Discretize(speeds, 0.02_s);
    auto modules = kinematics.ToSwerveModuleStates(speeds);
    kinematics.DesaturateWheelSpeeds(&modules, 4_mps);
    auto [fl, fr, bl, br] = modules;

    frc::Rotation2d flAngle{units::radian_t{rotfl.GetEncoder().GetPosition()}};
    frc::Rotation2d frAngle{units::radian_t{rotfr.GetEncoder().GetPosition()}};
    frc::Rotation2d blAngle{units::radian_t{rotbl.GetEncoder().GetPosition()}};
    frc::Rotation2d brAngle{units::radian_t{rotbr.GetEncoder().GetPosition()}};
    //SwerveModuleState::Optimize takes in the state, and the current angle, then optimizes the state to include that angle
    auto flOptimized = frc::SwerveModuleState::Optimize(fl, flAngle);
    auto frOptimized = frc::SwerveModuleState::Optimize(fr, frAngle);
    auto blOptimized = frc::SwerveModuleState::Optimize(bl, blAngle);
    auto brOptimized = frc::SwerveModuleState::Optimize(br, brAngle);


    //set state takes in the perfectly optimized state, the movement controller, and steering controller
    //optimized state contains two values, the speed and angle

    SetState(flOptimized, wheelfl, rotfl);
    SetState(frOptimized, wheelfr, rotfr);
    SetState(blOptimized, wheelbl, rotbl);
    SetState(brOptimized, wheelbr, rotbr);
  };

  pathplanner::RobotConfig config = pathplanner::RobotConfig::fromGUISettings();
  pathplanner::AutoBuilder::configure(
    [this]() { return poseEstimator->GetEstimatedPosition(); },
    [this](frc::Pose2d pose) {
        poseEstimator->ResetPosition(
            frc::Rotation2d{units::degree_t{ahrs->GetAngle()}},
            GetSwervePositions(),
            pose
        );
    },
    getRobotRelativeSpeeds,
    driveRobotRelative,
    std::make_shared<pathplanner::PPHolonomicDriveController>(
        pathplanner::PIDConstants(5.0, 0.0, 0.0),
        pathplanner::PIDConstants(5.0, 0.0, 0.0)
    ),
    config,
    []() {
        auto alliance = frc::DriverStation::GetAlliance();
        return alliance && alliance.value() == frc::DriverStation::Alliance::kRed;
    },
    nullptr
);

//should probably use sig figs but im scared
LimelightHelpers::setCameraPose_RobotSpace(
    "",
    0.2453941752,   // forward from center (positive = toward front)
    -0.23021288476,   // side offset (positive = left)
    0.29895206656,   // height from floor level of robot center
    0.0,   // roll
    0.0,  // pitch (negative = tilted down toward floor)
    0.0    // yaw (0 = facing forward)
);

}



void RobotPeriodic() {
  UpdatePose();
  LimelightHelpers::SetRobotOrientation(
    "",
    ahrs->GetYaw(),
    0, 0, 0, 0, 0 //sets yawrate, pitch, pitchrate, and roll to 0
  );

  LimelightHelpers::PoseEstimate mt2;

  //if(color == 'b'){
  mt2 = LimelightHelpers::getBotPoseEstimate_wpiBlue_MegaTag2("");
  //}
  /* else {
  mt2 = LimelightHelpers::getBotPoseEstimate_wpiRed_MegaTag2("limelight");
  } */

  //only trust megatag when there is at least one tag, and the robot isn't rotating at unreasonable speed
  bool isTrustworthy = mt2.tagCount >= 1 && std::abs(ahrs->GetRate()) < 720.0; 

  if (isTrustworthy) {
  poseEstimator->SetVisionMeasurementStdDevs(
    {0.01, 0.01, 686367.69} // x, y, theta (ignore vision rotation)
  );

  frc::SmartDashboard::PutBoolean("mt2 is trustworthy", isTrustworthy);

  /*if its trustworthy, add the pose estimated by megatag2 into our pose estimator
  It utilizes a kalman filter! In other words
  if vision stdev is small relative to gyro/encoder stdev, the robot snaps towards the vision estimate
  (favors vision)
  if its large, then the robot slowly drifts towards the vision estimate
  (favors encoders/gyro)
  */
  poseEstimator->AddVisionMeasurement(
    mt2.pose,
    mt2.timestampSeconds
  );


  }
  
  frc::Pose2d position = poseEstimator->GetEstimatedPosition();
  double yposition = position.Y().value();
  double xposition = position.X().value();
  double heading = position.Rotation().Degrees().value();
  frc::SmartDashboard::PutNumber("y position: ", yposition);
  frc::SmartDashboard::PutNumber("x position: ", xposition);
  frc::SmartDashboard::PutNumber("robot heading (degrees): ", heading);
  frc::SmartDashboard::PutNumber("robot heading (radians)", heading * PI / 180);

  frc::SmartDashboard::PutNumber("Raw Yaw: ", ahrs->GetYaw());
  frc::SmartDashboard::PutNumber("Raw Angle: ", ahrs->GetAngle());

  frc::SmartDashboard::PutNumber("encfl.Get", encfl.Get());
  frc::SmartDashboard::PutNumber("encfr.Get", encfr.Get());
  frc::SmartDashboard::PutNumber("encbl.Get", encbl.Get());
  frc::SmartDashboard::PutNumber("encbr.Get", encbr.Get());

  frc::SmartDashboard::PutNumber("Turret Heading (radians): ", HorizontalTurret.GetEncoder().GetPosition());
  frc::SmartDashboard::PutNumber("Turret Angle (revs): ", VerticalTurret.GetEncoder().GetPosition());
  frc::SmartDashboard::PutNumber("POV: ", controller.GetPOV());

  frc::SmartDashboard::PutBoolean("NavX Connected", ahrs->IsConnected());
  frc::SmartDashboard::PutBoolean("NavX Calibrating", ahrs->IsCalibrating());

  frc::Translation2d distance = GoalPosition - position.Translation();
  frc::SmartDashboard::PutNumber("Distance from goal (x): ", distance.X().value());
  frc::SmartDashboard::PutNumber("Distance from goal (y): ", distance.Y().value());
  frc::SmartDashboard::PutNumber("Distance from goal (overall): ", distance.Norm().value());
  

  //every 20ms, robot receives new data from limelight
  //currently not important for purposes of drive testing, but it will be in auto
  
  hastarget = LimelightHelpers::getTV("");
  tx = LimelightHelpers::getTX("");
  ty = LimelightHelpers::getTY("");
  ta = LimelightHelpers::getTA("");

  frc::SmartDashboard::PutNumber("HasTarget:", hastarget);
  frc::SmartDashboard::PutNumber("TX", tx);
  frc::SmartDashboard::PutNumber("TY", ty);
  frc::SmartDashboard::PutNumber("TA", ta);
  


  frc::SmartDashboard::PutNumber("Pose X (MT2): ", mt2.pose.X().value());
  frc::SmartDashboard::PutNumber("Pose Y (MT2): ", mt2.pose.Y().value());

  frc::SmartDashboard::PutNumber("MT2 tagCount", mt2.tagCount);
  frc::SmartDashboard::PutNumber("MT2 timestamp", mt2.timestampSeconds.value());
  frc::SmartDashboard::PutBoolean("isTrustworthy", isTrustworthy);

}


void AutonomousInit() {
  time.Reset();
  ResetGyro();
  ResetPoseEstimator();
  
  //defining my own shoot command
  frc2::CommandPtr shootCommand = 
  frc2::cmd::Run([this]() {
      pidfiresh.SetReference(6368, rev::spark::SparkBase::ControlType::kVelocity);
      Indexer.Set(IndexerSpeed);
  }).WithTimeout(2.0_s)
  .AndThen([this]() {
      firesh.StopMotor();
      Indexer.StopMotor();
  });


  frc2::CommandPtr intakeCommand = 
  frc2::cmd::Run([this]() {
      Intake.Set(-0.8);
  });

  frc2::CommandPtr alignTurretCommand = 
  frc2::cmd::Run([this]() {
      AlignTurret();
  });

  frc2::CommandPtr spinFlywheelCommand = 
  frc2::cmd::Run([this]() {
      pidfiresh.SetReference(6368, rev::spark::SparkBase::ControlType::kVelocity);
  });

  /*
  autoCommand = pathplanner::PathPlannerAuto("auto").ToPtr()
  .AndThen(std::move(shootCommand))
  .WithTimeout(14.5_s);
  */
 //load each path separately
  auto path1c = pathplanner::PathPlannerPath::fromPathFile("blue path 1 c");
  auto path1i = pathplanner::PathPlannerPath::fromPathFile("blue path 1 i");
  auto path1b = pathplanner::PathPlannerPath::fromPathFile("blue path 1 b");
  
  //first, path to center runs alone
  autoCommand = pathplanner::AutoBuilder::followPath(path1c)
  //next, the path to intake, as well as the intake commands run simultaneously
  .AndThen(pathplanner::AutoBuilder::followPath(path1i).DeadlineFor(std::move(intakeCommand)))
  //the robot takes the path back, while aligning the turret
  .AndThen(pathplanner::AutoBuilder::followPath(path1b).DeadlineFor(std::move(alignTurretCommand).AlongWith(std::move(spinFlywheelCommand))))
  //the robot shoots!
  .AndThen(std::move(shootCommand))
  //timeout to prevent hitting time limit
  .WithTimeout(14.5_s);
  autoCommand.Schedule();

  
  time.Start();

  
}

void AutonomousPeriodic() {
  frc2::CommandScheduler::GetInstance().Run();
}

void TeleopInit() {
  autoCommand.Cancel();
  time.Stop();
  time.Reset();
}


//every 20ms during teleop robot reads the joystick's percentages
void TeleopPeriodic() {

  //Resetting functionalities, MUST do at the start of every match
  //gyroscope resets when Y is pressed
  if(controller.GetYButtonPressed()){
    ResetPoseEstimator();
    ResetGyro();
  }

  if(controller.GetBButtonPressed()){
    VerticalTurret.GetEncoder().SetPosition(45);
    HorizontalTurret.GetEncoder().SetPosition(0);
  }


  //Sets turret position to zero, and limtis rotational movement.
  if (controller.GetPOV() == 90) {
    //right
    HorizontalTurret.Set(HorizontalSpeed);
  } else if (controller.GetPOV() == 270){
    //left
    HorizontalTurret.Set(-HorizontalSpeed);
  } else if (controller.GetPOV() == 180){
    //down TEMPORARY REMOVAL OF VERTICAL TURRET
    //VerticalTurret.Set(VerticalSpeed);
    Hang.Set(-HangSpeed);
  } else if (controller.GetPOV() == 0){
    //up TEMPORARY REMOVAL OF VERTICAL TURRET
    //VerticalTurret.Set(-VerticalSpeed);
    Hang.Set(HangSpeed);
  } else if (controller.GetStartButton()) {
    HorizontalTurret.GetEncoder().SetPosition(0);
  } else { //ensures autoalignment and manual turret movement are mutually exclusive
    if(controller.GetAButton()){
      AlignTurret();
    } else {
      HorizontalTurret.StopMotor();
      VerticalTurret.StopMotor();
      Hang.StopMotor();
    }

  }


  if (controller.GetXButton()){
    xstop();
    wheelfl.GetClosedLoopController().SetIAccum(0);
    wheelfr.GetClosedLoopController().SetIAccum(0);
    wheelbl.GetClosedLoopController().SetIAccum(0);
    wheelbr.GetClosedLoopController().SetIAccum(0);
  } else { //if else structure makes it so drive and x-stop are mutually exclusive
  //x, y, turn
  //for now, just calling drive on it's own
  //joysticks are inverted because wpi NWU axes co-ordinate system is weird, search it up if interested
  Drive(-controller.GetLeftY(), -controller.GetLeftX(), -controller.GetRightX());
  }



  //shooter code
  //actual rpm is targetrpm * 22/15
  double targetrpm = 6368;

  //if bumper is pressed, fire both motors at the target rpm, otherwise set their velocities to 0
  if(controller.GetRightBumper()){
  pidfiresh.SetReference(
      targetrpm,
      rev::spark::SparkBase::ControlType::kVelocity
  );
  } else {
    firesh.StopMotor();
  }
  


  //controller triggers set indexer velocity
  if(controller.GetLeftTriggerAxis()){
    Indexer.Set(-IndexerSpeed);
    Hopper.Set(HopperSpeed);
  } else if (controller.GetRightTriggerAxis()){
    Indexer.Set(IndexerSpeed);
    Hopper.Set(-HopperSpeed);
  } else {
    Indexer.StopMotor();
    Hopper.StopMotor();
  }


    //controller triggers set indexer velocity
  if(controller.GetLeftBumper()){
   Intake.Set(-0.7);
  } else {
    Intake.StopMotor();

  }



}

//helper function to align the turret
void AlignTurret(){
  //calculate distance from robot to goal
  frc::Translation2d poseTranslation = pose.Translation();
  frc::Translation2d distance = GoalPosition - poseTranslation;

  //calculate angle based on the x & y distances
  frc::Rotation2d angle = distance.Angle();

  frc::Rotation2d TurretTarget = angle - pose.Rotation();

  //normalizing the target angle to stay within soft limits
  double targetRad = TurretTarget.Radians().value();
  while (targetRad > PI)  targetRad -= 2.0 * PI;
  while (targetRad < -PI) targetRad += 2.0 * PI;

  //Sets the rotational motor's angle, to that position
  HorizontalTurret.GetClosedLoopController().SetReference(
    targetRad,
  rev::spark::SparkBase::ControlType::kPosition
  );

  //debugging utility
  frc::SmartDashboard::PutNumber("Turret Target (Rad)", TurretTarget.Radians().value());

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


  /* 
  since joysticks don't truly return to "zero", the deadbands (if statements) are there to ignore
  input IF the joystick values are negligible
  */

  x = frc::ApplyDeadband(x, 0.25);
  y = frc::ApplyDeadband(y, 0.25);
  rotate = frc::ApplyDeadband(rotate, 0.25);

  
  //rot2d reflects the AHRS gyroscope orientation
  frc::Rotation2d rot2d{units::degree_t{ahrs->GetAngle()}};

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

  //trying to increase speed
  units::radians_per_second_t rad{rotate*3};
  units::meters_per_second_t speedy{y*5};
  units::meters_per_second_t speedx{x*5};

  //speedx = (x * std::abs(x)) * 4_mps;

  /* ChassisSpeeds::FromFieldRelativeSpeeds takes in desired x, desired y, and angular velocities
  as well as the robots current angle
  */
  frc::ChassisSpeeds speeds = frc::ChassisSpeeds::FromFieldRelativeSpeeds(
    //uses the slewrate limiter to determine necessary chassis speeds
    //taking off speed limits, im curious...
    //limitx.Calculate(speedx),
    //limity.Calculate(speedy),
    speedx,
    speedy,
    rad,  // sensitivity multiplier?? increase if rotation is sluggish, decrease if jittery
    rot2d  // This is what enables field-oriented control
  );

  //converts the speeds to swerve module states

  speeds = frc::ChassisSpeeds::Discretize(speeds, 0.02_s);

  auto modules = kinematics.ToSwerveModuleStates(speeds);

  //safety to prevent wheels from spinning too fast
  kinematics.DesaturateWheelSpeeds(&modules, 5_mps);

  //just stores the swerve module states in each motor
  auto [fl, fr, bl, br] = modules;


  //experiment: getting wpilib to optimize angles instead (eliminating need for complex swerve math)
  frc::Rotation2d flAngle{units::radian_t{rotfl.GetEncoder().GetPosition()}};
  frc::Rotation2d frAngle{units::radian_t{rotfr.GetEncoder().GetPosition()}};
  frc::Rotation2d blAngle{units::radian_t{rotbl.GetEncoder().GetPosition()}};
  frc::Rotation2d brAngle{units::radian_t{rotbr.GetEncoder().GetPosition()}};


  //SwerveModuleState::Optimize takes in the state, and the current angle, then optimizes the state to include that angle
  auto flOptimized = frc::SwerveModuleState::Optimize(fl, flAngle);
  auto frOptimized = frc::SwerveModuleState::Optimize(fr, frAngle);
  auto blOptimized = frc::SwerveModuleState::Optimize(bl, blAngle);
  auto brOptimized = frc::SwerveModuleState::Optimize(br, brAngle);


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
  if(controller.GetRightBumper()){
  pidfiresh.SetReference(
      6368,
      rev::spark::SparkBase::ControlType::kVelocity
  );
  } else {
    firesh.StopMotor();
  }

  if(controller.GetLeftBumper()){
  pidfiresh2.SetReference(
      6368,
      rev::spark::SparkBase::ControlType::kVelocity
  );
  } else {
    firesh2.StopMotor();
  }
  
}

void SimulationInit() {}

void SimulationPeriodic() {}

//resets the gyroscope
void ResetGyro() {
  ahrs->ZeroYaw();
}

//reset odometry
void ResetPoseEstimator() {
  poseEstimator->ResetPosition(
  frc::Rotation2d{units::degree_t{ahrs->GetAngle()}}, 
  GetSwervePositions(),
  frc::Pose2d{0_m, 0_m, 0_rad}
  );

}

//helper function to update pose
void UpdatePose(){
  /*
  pose = odometry->Update(
    frc::Rotation2d{units::degree_t{ahrs->GetYaw()}},
    GetSwervePositions()
  );
  */

  pose = poseEstimator->Update(
    frc::Rotation2d{units::degree_t{ahrs->GetAngle()}},
    GetSwervePositions()
  );

}

//helper function to get the positions of each swerve module
wpi::array<frc::SwerveModulePosition, 4> GetSwervePositions(){
  return {
    frc::SwerveModulePosition{
      units::meter_t{wheelfl.GetEncoder().GetPosition()}, 
      frc::Rotation2d{units::radian_t{rotfl.GetEncoder().GetPosition()}}
  },

  frc::SwerveModulePosition{
      units::meter_t{wheelfr.GetEncoder().GetPosition()}, 
      frc::Rotation2d{units::radian_t{rotfr.GetEncoder().GetPosition()}}
  },

  frc::SwerveModulePosition{
      units::meter_t{wheelbl.GetEncoder().GetPosition()}, 
      frc::Rotation2d{units::radian_t{rotbl.GetEncoder().GetPosition()}}
  },

  frc::SwerveModulePosition{
      units::meter_t{wheelbr.GetEncoder().GetPosition()}, 
      frc::Rotation2d{units::radian_t{rotbr.GetEncoder().GetPosition()}}
  }
};



}


void xstop(){
  //just tells the robot which angle to set everything to in order to make an x :)
  rotfl.GetClosedLoopController().SetReference(PI/4,  rev::spark::SparkBase::ControlType::kPosition);
  rotfr.GetClosedLoopController().SetReference(-PI/4, rev::spark::SparkBase::ControlType::kPosition);
  rotbl.GetClosedLoopController().SetReference(-PI/4, rev::spark::SparkBase::ControlType::kPosition);
  rotbr.GetClosedLoopController().SetReference(PI/4,  rev::spark::SparkBase::ControlType::kPosition);

  //tells robot to hold a velocity of 0 on all motors
  wheelfl.GetClosedLoopController().SetReference(0, rev::spark::SparkBase::ControlType::kVelocity);
  wheelfr.GetClosedLoopController().SetReference(0, rev::spark::SparkBase::ControlType::kVelocity);
  wheelbl.GetClosedLoopController().SetReference(0, rev::spark::SparkBase::ControlType::kVelocity);
  wheelbr.GetClosedLoopController().SetReference(0, rev::spark::SparkBase::ControlType::kVelocity);

  //yes the repeated GetClosedLoopController() is ugly
  //no you cannot store the closed loop controllers as variables
  //actually you can. note: simplify ts later 
}
};

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
