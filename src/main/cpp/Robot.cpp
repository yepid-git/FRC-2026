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
//#include <units/angle.h>
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
#include <vector>
#include <utility>

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
  //color
  bool isRed = false;

  //limelight placeholder variables
  bool hastarget = false;
  double tx;
  double ty;
  double ta;

  //vertical and horizontal turret
  double VerticalSpeed = 0.1; 
  double HorizontalSpeed = 0.1;
  double IndexerSpeed = 1;
  double HopperSpeed = 0.4; 
  double HangSpeed = 0.5;
  double IntakeSpeed = 0.7;

  //member for the last known angle, avoid bad gyro readings during disconnections (if happens)
  double lastKnownAngle = 0.0;
  double lastKnownYaw = 0.0;  


  double drivespeed = 5;

  //member for the auto command
  frc2::CommandPtr autoCommand = frc2::cmd::None();

  //placeholder variable for the goal hub's position on the field
  frc::Translation2d RedGoalPosition{11.612_m, 4.5_m};
  frc::Translation2d BlueGoalPosition{4.612_m, 4.5_m};
  frc::Translation2d GoalPosition{};


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
  frc::XboxController controller{0}; //1st controller: drive, intake, indexer, reset functionalities
  frc::XboxController controller2{1}; //2nd controller: shoot wheels, horizontal & vertical turret, auto turret, hopper (forwards/backwards)

  //limits acceleration to 6m/s^2
  frc::SlewRateLimiter<units::meters_per_second> limitx{3_mps / .5_s};
  frc::SlewRateLimiter<units::meters_per_second> limity{3_mps / .5_s};
  frc::SlewRateLimiter<units::radians_per_second> limitrot{3_rad_per_s / .5_s};
  //frc::SlewRateLimiter<units::radian_t> turretlimit{3_rad / 1_s};
  frc::SlewRateLimiter<units::radians> turretlimit{0.5_rad / 1_s};
  //frc::SlewRateLimiter<double> limitturretturn{3.0}; // 3 radians per second (i think - Alex Wang 0314)

  //Controller Mode Variables
  bool m_manual_mode = true;

  //declaring odometry object at class level using pointer, to avoid scoping issues
  //std::unique_ptr<frc::SwerveDriveOdometry<4>> odometry; swapped for pose estimator!
  std::unique_ptr<frc::SwerveDrivePoseEstimator<4>> poseEstimator;
  frc::Pose2d pose;


void RobotInit(){


  LimelightHelpers::setPipelineIndex("", 0);
  //i dont think the led is necessary im ngl
  LimelightHelpers::setLEDMode_ForceOff("");

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
    .PositionConversionFactor((PI * 0.0965 * 1.1) / gearratio)
    .VelocityConversionFactor(((PI * 0.0965 * 1.1) / gearratio) / 60.0);

  driveConfig.closedLoop
    .SetFeedbackSensor(rev::spark::FeedbackSensor::kPrimaryEncoder)
    //pid might be too small?
    .Pid(0.2, 0.000001, 0.00000001)
//    .VelocityFF(0.25)
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
    .VoltageCompensation(12.0)
    .Inverted(true)
    .OpenLoopRampRate(0.4) // seconds to full power
    .ClosedLoopRampRate(0.4)
    .SmartCurrentLimit(60)
    .SetIdleMode(rev::spark::SparkBaseConfig::IdleMode::kCoast);
  
  shooterLeaderConfig.closedLoop
    .SetFeedbackSensor(rev::spark::FeedbackSensor::kPrimaryEncoder)
    .Pid(0.001, 0.0, 0.0)
    .VelocityFF(0.000147)
    //limit voltage
    //.OutputRange(-0.5, 0.5)
    .IZone(0);

  //sets the follower shooter to actually follow the leader
  shooterFollowerConfig
  .SetIdleMode(rev::spark::SparkBaseConfig::IdleMode::kCoast)
  .Follow(firesh, true);

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
  .OutputRange(-0.9, 0.9);

  HorizontalTurretConfig.softLimit
    //.ForwardSoftLimit((-PI/2) + 3.318 - 0.1)  // 180*0.95 degrees
    .ForwardSoftLimit(PI/2)
    .ForwardSoftLimitEnabled(true)
    //.ReverseSoftLimit((-PI/2)-0.789 + 0.1) // before
    .ReverseSoftLimit((-3*PI)/4) 
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


  IndexerConfig
    .Inverted(false) // flip to true if it runs backwards
    .SetIdleMode(rev::spark::SparkBaseConfig::IdleMode::kBrake);
  
  IndexerConfig.closedLoop
  .SetFeedbackSensor(rev::spark::FeedbackSensor::kPrimaryEncoder)
  .Pid(0.3, 0.0, 0.0)
  .PositionWrappingEnabled(false);

  IntakeConfig.closedLoop
  .SetFeedbackSensor(rev::spark::FeedbackSensor::kPrimaryEncoder)
  .Pid(0.3, 0.0, 0.0)
  .PositionWrappingEnabled(false)
  .OutputRange(-0.75, 0.75);


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



  /*
  CURRENT LIMITS ON EVERYTHING!!!
  if we still have brownouts, lets try uncommenting!
  driveConfig.SmartCurrentLimit(40);        // each drive wheel
  steerConfig.SmartCurrentLimit(20);        // each steer motor
  IndexerConfig.SmartCurrentLimit(20);
  IntakeConfig.SmartCurrentLimit(30);
  HangConfig.SmartCurrentLimit(40);
  hopperConfig.SmartCurrentLimit(20);
  HorizontalTurretConfig.SmartCurrentLimit(15);
  VerticalTurretConfig.SmartCurrentLimit(15);
  
  
  */


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
  double broff = -0.245;


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
            frc::Rotation2d{units::degree_t{lastKnownAngle}},
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
    180    // yaw (0 = facing forward)
);

}



void RobotPeriodic() {
  GetSafeRotation(); // updates lastKnownAngle cache
  UpdatePose();
  LimelightHelpers::SetRobotOrientation(
    "",
    lastKnownAngle,
    0, 0, 0, 0, 0 //sets yawrate, pitch, pitchrate, and roll to 0
  );

  LimelightHelpers::PoseEstimate testPose = isRed ?
    LimelightHelpers::getBotPoseEstimate_wpiRed_MegaTag2("") :
    LimelightHelpers::getBotPoseEstimate_wpiBlue_MegaTag2("");
  frc::SmartDashboard::PutBoolean("Limelight Reachable: ", testPose.timestampSeconds.value() > 0);

  LimelightHelpers::PoseEstimate mt2;

  //if(color == 'b'){
  mt2 = isRed ?
    LimelightHelpers::getBotPoseEstimate_wpiRed_MegaTag2("") :
    LimelightHelpers::getBotPoseEstimate_wpiBlue_MegaTag2("");
  //}
  /* else {
  mt2 = LimelightHelpers::getBotPoseEstimate_wpiRed_MegaTag2("limelight");
  } */

  //only trust megatag when there is at least one tag, and the robot isn't rotating at unreasonable speed
  //also ensures megatag is trustworthy only when the gyro is healthy
  bool gyroHealthy = ahrs->IsConnected() && !ahrs->IsCalibrating();
  bool isTrustworthy = mt2.tagCount >= 1 && std::abs(ahrs->GetRate()) < 720.0 && gyroHealthy; 

  if (isTrustworthy) {
  poseEstimator->SetVisionMeasurementStdDevs(
    {0.5, 0.5, 686367.69} // x, y, theta (ignore vision rotation)
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

  frc::Translation2d distance =  position.Translation() - GoalPosition;
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

  auto alliance = frc::DriverStation::GetAlliance();
  //handles flipping of coordinates 
  if (alliance && alliance.value() == frc::DriverStation::Alliance::kRed) {
    isRed = true;
    GoalPosition = RedGoalPosition;
  } else {
    GoalPosition = BlueGoalPosition;
    isRed = false;
  }
  //zeros out gyro, and then ensures that the ahrs reads 180 degrees (it starts backwards)
  ahrs->ZeroYaw();
  if (isRed) {
      ahrs->SetAngleAdjustment(0.0);
      lastKnownAngle = 0.0;
  } else {
      ahrs->SetAngleAdjustment(180.0);
      lastKnownAngle = 180.0;
  }
  lastKnownYaw = 180.0;

  ResetPoseFromLimelight();

  VerticalTurret.GetEncoder().SetPosition(45);
  HorizontalTurret.GetEncoder().SetPosition(-PI/2);

  //color change
  time.Reset();

auto makeShootCommand = [this]() {
  return frc2::cmd::Run([this]() {
      pidfiresh.SetReference(1700, rev::spark::SparkBase::ControlType::kVelocity);
      Indexer.Set(IndexerSpeed);
  }).WithTimeout(5.0_s).AndThen([this]() {
      firesh.StopMotor(); Indexer.StopMotor();
  });
};


auto makeIntakeCommand = [this]() {
  return frc2::cmd::Run([this]() {
      Intake.Set(-1);
  });
};

auto makeAlignTurretCommand = [this]() {
  return frc2::cmd::Run([this]() {
      AlignTurret();
  }).WithTimeout(0.5_s);
};

auto makeSpinFlywheelCommand = [this]() {
  return frc2::cmd::Run([this]() {
      pidfiresh.SetReference(1700, rev::spark::SparkBase::ControlType::kVelocity);
  });
};



auto waitForVision = [this]() {
return frc2::cmd::WaitUntil([this]() {
LimelightHelpers::PoseEstimate mt2 = isRed ?
  LimelightHelpers::getBotPoseEstimate_wpiRed_MegaTag2("") :
  LimelightHelpers::getBotPoseEstimate_wpiBlue_MegaTag2("");
return mt2.tagCount >= 1;
}).WithTimeout(0.5_s)
.AndThen(frc2::cmd::RunOnce([this]() {
ResetPoseFromLimelight();
// Tighten vision trust right at the start since we have a clear tag view
poseEstimator->SetVisionMeasurementStdDevs({0.1, 0.1, 686367.69});
}));
};

  /*
  autoCommand = pathplanner::PathPlannerAuto("auto").ToPtr()
  .AndThen(std::move(shootCommand))
  .WithTimeout(14.5_s);
  */
 //load each path separately
  auto path1c = pathplanner::PathPlannerPath::fromPathFile("blue path 1 c");
  auto path1i = pathplanner::PathPlannerPath::fromPathFile("blue path 1 i");
  auto path1b = pathplanner::PathPlannerPath::fromPathFile("blue path 1 b");
  
  //lambda function to mirror start pose
  frc::Pose2d startPose = isRed ?
    frc::Pose2d{13.03_m, 7.459_m, frc::Rotation2d{0_deg}} :
    frc::Pose2d{3.506_m, 7.459_m, frc::Rotation2d{180_deg}};

  //first, path to center runs alone
  autoCommand = 
        pathplanner::AutoBuilder::resetOdom(startPose)
        .AndThen(makeShootCommand().DeadlineFor(waitForVision()))
        .AndThen(pathplanner::AutoBuilder::followPath(path1c))
        .AndThen(pathplanner::AutoBuilder::followPath(path1i).DeadlineFor(makeIntakeCommand()))
        .AndThen(pathplanner::AutoBuilder::followPath(path1b).DeadlineFor(makeSpinFlywheelCommand()))
        .AndThen(makeShootCommand())
        .WithTimeout(14.5_s);
  autoCommand.Schedule();

  
  time.Start();

  
}

void AutonomousPeriodic() {
  /*
  frc::Timer m_timer;
  m_timer.Reset();
  
  
  if(5.0_s < m_timer.Get()){
    pidfiresh.SetReference(1700, rev::spark::SparkBase::ControlType::kVelocity);
    if(1.0_s < m_timer.Get()){
    HorizontalTurret.GetClosedLoopController().SetReference(-PI/4+PI/12, rev::spark::SparkBase::ControlType::kPosition);
    Indexer.Set(IndexerSpeed);
    }
  } else {
      Drive(1, 0, 0);
  }
  

  Indexer.Set(IndexerSpeed);
  pidfiresh.SetReference(1800, rev::spark::SparkBase::ControlType::kVelocity);
  frc::SmartDashboard::PutNumber("time: ", m_timer.Get().value());

  */

  frc2::CommandScheduler::GetInstance().Run();
}

void TeleopInit() {
  autoCommand.Cancel();
  ahrs->SetAngleAdjustment(0.0);

  //set trust in vision back to normal
  poseEstimator->SetVisionMeasurementStdDevs({0.5, 0.5, 686367.69});
  //color change
  auto alliance = frc::DriverStation::GetAlliance();
  //handles flipping of coordinates 
  if (alliance && alliance.value() == frc::DriverStation::Alliance::kRed) {
    GoalPosition = RedGoalPosition;
    isRed = true;
  } else {
    GoalPosition = BlueGoalPosition;
    isRed = false;
  }
  time.Stop();
  time.Reset();
}


//every 20ms during teleop robot reads the joystick's percentages
void TeleopPeriodic() {

  //Resetting functionalities, MUST do at the start of every match
  //gyroscope resets when Y is pressed
  //moved into start of auto but kept here if needed during tele
  
  if(controller.GetYButtonPressed()){
    //resets gyro heading, make sure robot is straight before doing this
    ResetGyro();
    ResetPoseFromLimelight();
    VerticalTurret.GetEncoder().SetPosition(45);
    HorizontalTurret.GetEncoder().SetPosition(-PI/2);
  }

  /*
  if(controller.GetBButtonPressed()){
    VerticalTurret.GetEncoder().SetPosition(45);
    HorizontalTurret.GetEncoder().SetPosition(-PI/2+PI/4); //starting position is -90 degrees MIGHT CHANGE
  }
  */

  //Sets turret position to zero, and limtis rotational movement.
  if (controller2.GetPOV() == 90) {
    //right
    HorizontalTurret.Set(HorizontalSpeed);
  } else if (controller2.GetPOV() == 270){
    //left
    HorizontalTurret.Set(-HorizontalSpeed);
  } else if (controller2.GetPOV() == 180){
    //down TEMPORARY REMOVAL OF VERTICAL TURRET
    VerticalTurret.Set(VerticalSpeed);
    //Hang.Set(-HangSpeed);
  } else if (controller2.GetPOV() == 0){
    //up TEMPORARY REMOVAL OF VERTICAL TURRET
    VerticalTurret.Set(-VerticalSpeed);
    //Hang.Set(HangSpeed);
  } else { //ensures autoalignment and manual turret movement are mutually exclusive
    if(controller2.GetAButton()){
      AlignTurret();
    } else {
      HorizontalTurret.StopMotor();
      VerticalTurret.StopMotor();
      Hang.StopMotor();
    }
  }


  //hopper code
  if (controller2.GetRightTriggerAxis()){
    Hopper.Set(HopperSpeed);
  } 
  if (controller2.GetLeftTriggerAxis()){
    Hopper.Set(-HopperSpeed);
  } 
  if (!controller2.GetLeftTriggerAxis() && !controller2.GetRightTriggerAxis()) {
    Hopper.StopMotor();
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


  // if (controller.GetLeftStickButton()){
  //   drivespeed = 10;
  // }
  // if(controller.GetRightStickButton()){
  //   drivespeed = 5;
  // }

  //shooter code
  //actual rpm is targetrpm * 22/15
  double targetrpm = 1700;

  //if bumper is pressed, fire both motors at the target rpm, otherwise set their velocities to 0
  if(controller2.GetXButton()){
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
  } else if (controller.GetRightTriggerAxis()){
    Indexer.Set(IndexerSpeed);
  } else {
    Indexer.StopMotor();
  }


  //controller triggers set indexer velocity
  if(controller.GetLeftBumper()){
   Intake.Set(-IntakeSpeed);
  } else if (controller.GetRightBumper()){
    Intake.Set(IntakeSpeed);
  } else { 
    Intake.StopMotor();
  }



}

//helper function to align the turret
void AlignTurret(){
  //calculate distance from robot to goal
  frc::Translation2d poseTranslation = pose.Translation();
  frc::Translation2d distance = poseTranslation - GoalPosition;

  //calculate angle based on the x & y distances
  frc::Rotation2d angle = distance.Angle();

  //calculate the vertical angle of the turret needed for the distance
  double targetVertical = extrapolateAngle(distance.Norm().value());

  frc::Rotation2d TurretTarget = angle - pose.Rotation();

  frc::SmartDashboard::PutNumber("Turret Target: ", TurretTarget.Radians().value());

  //normalizing the target angle to stay within soft limits
  double targetRad = -TurretTarget.Radians().value();
  while (targetRad > PI)  targetRad -= 2.0 * PI;
  while (targetRad < -PI) targetRad += 2.0 * PI;

  //clamps the value to the robots softlimits
  targetRad = std::clamp(targetRad, -PI * 0.95, PI * 0.95);

  //insert slew rate limiter!
  //targetRad = limitturretturn.Calculate(units::radian_t(targetRad));
  units::radian_t targetRad_unit{targetRad};

  frc::SmartDashboard::PutNumber("Target Rad: ", targetRad);
  frc::SmartDashboard::PutNumber("Butt: ", double(turretlimit.Calculate(targetRad_unit)));

  //Sets the rotational motor's angle, to that position
  HorizontalTurret.GetClosedLoopController().SetReference(
    //turretlimit.Calculate(units::radians{targetRad}),
    double(turretlimit.Calculate(targetRad_unit)),
    //targetRad,
    rev::spark::SparkBase::ControlType::kPosition
  );

  //VerticalTurret.GetClosedLoopController().SetReference(
    //targetVertical,
    //rev::spark::SparkBase::ControlType::kPosition
  //);

  //debugging utility
  frc::SmartDashboard::PutNumber("Turret Target (Rad)", targetRad);

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
  frc::Rotation2d rot2d{units::degree_t{lastKnownAngle}};

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
  units::radians_per_second_t rad{rotate*drivespeed};
  units::meters_per_second_t speedy{y*drivespeed};
  units::meters_per_second_t speedx{x*drivespeed};

  //speedx = (x * std::abs(x)) * 4_mps;

  /* ChassisSpeeds::FromFieldRelativeSpeeds takes in desired x, desired y, and angular velocities
  as well as the robots current angle
  */
  frc::ChassisSpeeds speeds = frc::ChassisSpeeds::FromFieldRelativeSpeeds(
    //uses the slewrate limiter to determine necessary chassis speeds
    //taking off speed limits, im curious...
    limitx.Calculate(speedx),
    limity.Calculate(speedy),
    //speedx,
    //speedy,
    rad*1.5,  // sensitivity multiplier?? increase if rotation is sluggish, decrease if jittery
    rot2d  // This is what enables field-oriented control
  );

  //converts the speeds to swerve module states

  speeds = frc::ChassisSpeeds::Discretize(speeds, 0.02_s);

  auto modules = kinematics.ToSwerveModuleStates(speeds);

  //safety to prevent wheels from spinning too fast
  //kinematics.DesaturateWheelSpeeds(&modules, 5_mps);

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
  frc::Rotation2d{units::degree_t{lastKnownAngle}}, 
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
    frc::Rotation2d{units::degree_t{lastKnownAngle}},
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

//helper function to reset pose based on limelight
void ResetPoseFromLimelight() {
LimelightHelpers::PoseEstimate llPose = isRed ?
    LimelightHelpers::getBotPoseEstimate_wpiRed_MegaTag2("") :
    LimelightHelpers::getBotPoseEstimate_wpiBlue_MegaTag2("");

frc::SmartDashboard::PutBoolean("Pose Reset Used Vision: ", llPose.tagCount >= 1);

frc::Pose2d fallback = isRed ?
    frc::Pose2d{13.03_m, 7.459_m, frc::Rotation2d{0_deg}} :
    frc::Pose2d{3.506_m, 7.459_m, frc::Rotation2d{180_deg}};

poseEstimator->ResetPosition(
    frc::Rotation2d{units::degree_t{lastKnownAngle}},
    GetSwervePositions(),
    llPose.tagCount >= 1 ? llPose.pose : fallback
);
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

}

//navx safeguard
frc::Rotation2d GetSafeRotation() {
    if (ahrs->IsConnected() && !ahrs->IsCalibrating()) {
        lastKnownAngle = ahrs->GetAngle();
        lastKnownYaw = ahrs->GetYaw(); 
    }
    return frc::Rotation2d{units::degree_t{lastKnownAngle}};
}


//distance angle table
//distances go first, angles go second
//dummy values for now, also MUST be sorted
std::vector<std::pair<double, double>> distanceAngleTable = {
  {1.0,  55.0},
  {2.0,  50.0},
  {3.0,  45.0},
};

double extrapolateAngle(double d){
  //unlikely the list is empty, but just to be safe!
  if (distanceAngleTable.empty()) return 45.0;

  //used for out of bounds angle handling
  if (d <= distanceAngleTable.front().first) return distanceAngleTable.front().second;
  if (d >= distanceAngleTable.back().first)  return distanceAngleTable.back().second;

  //search for the the points the distance is between
  for (size_t i = 0; i + 1 < distanceAngleTable.size(); i++) {
    double d0 = distanceAngleTable[i].first;
    double d1 = distanceAngleTable[i + 1].first;

    if (d >= d0 && d <= d1) {
      // How far between d0 and d1 are we? (0.0 to 1.0)
      double t = (d - d0) / (d1 - d0);
      double a0 = distanceAngleTable[i].second;
      double a1 = distanceAngleTable[i + 1].second;
      //returns an average of the angles at the two points
      return a0 + t * (a1 - a0);
    }
  }
  
  //returns the last angle if the value isn't between any known distances
  return distanceAngleTable.back().second;
}

//sets position of the horizontal/vertical turret and resets everything
void resetAll(){
  ResetGyro();
  ResetPoseFromLimelight();
  VerticalTurret.GetEncoder().SetPosition(45);
  HorizontalTurret.GetEncoder().SetPosition(-PI/2+PI/4);
}

};

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif