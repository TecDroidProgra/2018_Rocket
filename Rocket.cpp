/*Esteban Padilla Cerdio
April 25,2018*/

#include <Encoder.h>
#include <IterativeRobot.h>
#include <Joystick.h>
#include <LiveWindow/LiveWindow.h>
#include <RobotDrive.h>
#include <Timer.h>
#include <WPILib.h>
#include <ADXRS450_Gyro.h>
#include <PIDController.h>
#include <PIDOutput.h>
#include <math.h>
#include <SampleRobot.h>
#include <Solenoid.h>
#define PI 3.141592653589793238
#define REC 0.75

class Robot: public frc::IterativeRobot {
public:
    Robot() {
      m_encoder.SetDistancePerPulse(PI*6/2048);
      CameraServer::GetInstance()->StartAutomaticCapture();
      cs::UsbCamera camera = CameraServer::GetInstance()->StartAutomaticCapture();
      camera.SetResolution(640, 480);
    }

private:
  double SENSIBILIDAD = 0.75;
  //Autonomous

  frc::Encoder m_encoder{0, 1, false, Encoder::EncodingType::k4X};
  frc::ADXRS450_Gyro gyro;
  frc::Timer timer;
  std::string gameData;
  double gyroAngle, encoderDistance, setAngle;
  double scaleMov[10]={215.0,35.0,3.6,50.0,0.0,0.5,0.0,-20.0,0.0}; //Avanzar a Scale desde un lado
  int scaleMovType[10]={1,2,3,5,0,3,4,1,6};
  double strMov[4]={0.0,170.0,0.0}; //Avanzar recto
  int strMovType[4]={0,1,6};
  double centMov[10]={0.0,24.0,-90.0,55.0,90.0,3.6,75.0,0.0,1.0,0.0};  //Avanzar a switch desde centro
  int centMovType[10]={0,1,2,1,2,3,5,4,3,6};
  double sameMov[11]={0.0,170,3.6,-90,0.5,90.0,25.0,-90.0,150.0,90.0,0.0}; //Avanzar a switch desde mismo lado
  int sameMovType[11]={0,1,3,2,4,2,1,2,1,2,6};
  double difMov[6]={215.0,-85.0,130.0,47.0,0.0}; //Avanzar al scale del lado opuesto
  int difMovType[6]={1,2,1,2,6};
  int move,type,movement;
  char side;
  bool isPIDActive = false;
  static constexpr double kP = 1;
  static constexpr double kI = 0.8;
  static constexpr double kD = 0.5;
  static constexpr double kPEncoder = 1;
  static constexpr double kIEncoder = 0.8;
  static constexpr double kDEncoder = 0.5;

  //Teleoperated

  frc::XboxController mando{ 0 };
  frc::VictorSP elevador  { 4 };
  frc::VictorSP amigo { 5 };
  frc::VictorSP m_frontLeft{ 0 };
  frc::VictorSP m_rearLeft{ 1 };
  frc::VictorSP esc1{ 7 };
  frc::VictorSP esc2{ 8 };
  frc::SpeedControllerGroup m_left{m_frontLeft, m_rearLeft};
  frc::VictorSP m_frontRight{ 2 };
  frc::VictorSP m_rearRight{ 3 };
  frc::SpeedControllerGroup m_right{m_frontRight, m_rearRight};
  frc::DifferentialDrive drive{m_left, m_right};
  frc::Joystick m_stick{ 0 };
  frc::DoubleSolenoid DoubleSolenoid1 { 0, 1 };
  frc::DoubleSolenoid DoubleSolenoid2 { 2, 3 };
  bool isClawOpen, isClawDown,isDownActive,isOpenActive;
  //

void moveForward(double distance, bool withScale){
  if(!isPIDActive){
    m_encoder.Reset();
    pidControllerEncoder.SetSetpoint(distance);
  }
  encoderDistance = m_encoder.GetDistance();
  if(fabs(pidControllerEncoder.GetSetpoint()-encoderDistance)>0.1){
    if(withScale)
    elevador.Set(1);
    frc::SmartDashboard::PutNumber("Encoderi", m_encoder.GetDistance());
    pidControllerEncoder.Enable();
    isPIDActive = true;
    }
  else{
    pidControllerEncoder.Disable();
    move+=1;
    isPIDActive = false;
  }
}

void turnAngle(double angle){
  if(!isPIDActive){
    gyro.Reset();
    pidControllerGyro.SetSetpoint(angle);
  }
  gyroAngle = gyro.GetAngle();
  if(fabs(pidControllerGyro.GetSetpoint()-gyroAngle)>0.1){
    frc::SmartDashboard::PutNumber("Gyro",(int)gyroAngle%360);
    pidControllerGyro.Enable();
    isPIDActive = true;
  }
  else{
    move+=1;
    pidControllerGyro.Disable();
    isPIDActive = false;
  }
}

void activateMotors(double seconds){
  if(!isPIDActive){
    timer.Reset();
    timer.Start();
  }
  elevador.Set(0.75);
  isPIDActive = true;
  frc::SmartDashboard::PutNumber("Time",timer.Get());
  if(timer.Get()>seconds){
    elevador.Set(0);
    timer.Stop();
    move+=1;
    isPIDActive = false;
  }
}

class GyroOutput : public frc::PIDOutput {
  public:
    explicit GyroOutput(frc::DifferentialDrive& r)
    :m_rd(r) {
      m_rd.SetSafetyEnabled(false);
    }

    void PIDWrite(double output) override {
      m_rd.ArcadeDrive(0, output);
    }

  private:
    frc::DifferentialDrive& m_rd;
};

class EncoderOutput : public frc::PIDOutput {
  public:
    explicit EncoderOutput(frc::DifferentialDrive& r)
      : m_rd2(r) {
      m_rd2.SetSafetyEnabled(false);
    }

    void PIDWrite(double output) override {
      m_rd2.ArcadeDrive(output, 0);
    }

  private:
    frc::DifferentialDrive& m_rd2;
};

EncoderOutput *mEncoderOutput = new EncoderOutput(drive);
GyroOutput *mGyroOutput = new GyroOutput(drive);
frc::PIDController pidControllerGyro { kP, kI, kD, &gyro,mGyroOutput};
frc::PIDController pidControllerEncoder { kPEncoder, kIEncoder, kDEncoder, &m_encoder,mEncoderOutput};

void AutonomousInit() override {
  //CAMBIAR AQUÍ DEPENDIENDO DEL LADO: 'C':Centro 'L':Izquierda 'D':Derecha 'S': Straight
  side = 'R';

  pidControllerGyro.SetInputRange(-360,360);
  pidControllerGyro.SetOutputRange(-0.6, 0.6);
  pidControllerEncoder.SetInputRange(-250,250);
  pidControllerEncoder.SetOutputRange(-0.6, 0.6);
  move = 0;
  pidControllerGyro.Disable();
  pidControllerEncoder.Disable();
  gyro.Reset();
  m_encoder.Reset();
  isPIDActive = false;
  gameData = frc::DriverStation::GetInstance().GetGameSpecificMessage();
  setAngle = 0;
  timer.Reset();
  isClawDown = false;
  isClawOpen = false;
  isDownActive = false;
  isOpenActive = false;
  }

void AutonomousPeriodic() override {
  if(side=='C'){
    type = centMovType[move];
    movement = centMov[move];
    if(gameData.at(0)=='R')
      setAngle=-movement;
    else
      setAngle=movement;
  }
  else if(side=='S'){
    type = strMovType[move];
    movement = strMov[move];
  }
  else{
    if(gameData.at(1)==side){
      type = scaleMovType[move];
      movement = scaleMov[move];
    }
    else{
      type = difMovType[move];
      movement=difMov[move];
    }
    if(gameData.at(1)=='R')
      setAngle=-movement;
    else
      setAngle=movement;
  }

  switch(type){
    case 0: //Bajar pinza
      DoubleSolenoid2.Set(frc::DoubleSolenoid::kForward);
      isClawDown=true;
      move+=1;
      break;
    case 1: //Avanzar
      moveForward(movement,false);
      break;
    case 2: //Girar
      turnAngle(setAngle);
      break;
    case 3: //Levantar elevador
      activateMotors(movement);
      break;
    case 4: //Abrir pinza
      DoubleSolenoid1.Set(frc::DoubleSolenoid::kReverse);
      isClawOpen=true;
      activateMotors(movement);
      break;
    case 5: //Avanzar y subir el elevador
      moveForward(movement,true);
      break;
  };
}

void TeleopInit() override {
  gyro.Reset();
  m_encoder.Reset();
  pidControllerGyro.Disable();
  pidControllerEncoder.Disable();
  DoubleSolenoid1.Set(frc::DoubleSolenoid::kOff);
  DoubleSolenoid2.Set(frc::DoubleSolenoid::kOff);
}

void TeleopPeriodic() override {
  drive.ArcadeDrive(-mando.GetRawAxis(1)*SENSIBILIDAD,mando.GetRawAxis(0)*0.75);
  esc1.Set(-mando.GetRawAxis(5));
  esc2.Set(mando.GetRawAxis(5));
  amigo.Set(mando.GetRawAxis(4));

  frc::SmartDashboard::PutNumber("Gyro",(int)gyro.GetAngle()%360);
  frc::SmartDashboard::PutNumber("Encoderi", m_encoder.GetDistance());

  elevador.Set(mando.GetRawAxis(3)-mando.GetRawAxis(2)*0.75);

  if (mando.GetRawButton(1)&&!isClawOpen) {
    DoubleSolenoid1.Set(frc::DoubleSolenoid::kForward);
    isOpenActive=true;
  }
  else if (mando.GetRawButton(1)&&isClawOpen) {
    DoubleSolenoid1.Set(frc::DoubleSolenoid::kReverse);
    isOpenActive=false;
  }
  else {
    DoubleSolenoid1.Set(frc::DoubleSolenoid::kOff);
    if (isOpenActive)
      isClawOpen = true;
    else
      isClawOpen= false;
    }

  if (mando.GetRawButton(4)&&!isClawDown) {
    DoubleSolenoid2.Set(frc::DoubleSolenoid::kForward);
    isDownActive=true;
  }
  else if (mando.GetRawButton(4)&&isClawDown) {
    DoubleSolenoid2.Set(frc::DoubleSolenoid::kReverse);
    isDownActive=false;
  }
  else {
    DoubleSolenoid2.Set(frc::DoubleSolenoid::kOff);
    if (isDownActive)
      isClawDown = true;
    else
      isClawDown= false;
  }

  if (mando.GetRawButton(6))
    SENSIBILIDAD = 0.75;
  else if (mando.GetRawButton(5))
    SENSIBILIDAD = 10;
}

START_ROBOT_CLASS(Robot)


