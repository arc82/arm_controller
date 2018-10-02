#include <Servo.h>
#include <Wire.h>
#include <arc.h>

class Vec2d {
public:
  Vec2d();
  Vec2d(float ix, float iy);

  float dot(const Vec2d& other);
  bool isCWRotation(const Vec2d& v); //Returns true if you need to rotate CW from v to get to this vector
  float magnitude();
  friend Vec2d operator+(const Vec2d& v1, const Vec2d& v2);
  friend Vec2d operator-(const Vec2d& v1, const Vec2d& v2);
  friend Vec2d operator*(const float& scalar, const Vec2d& v);
  friend Vec2d operator*(const Vec2d& v, const float& scalar);


  float x;
  float y;
};

class Arm {
public:
  Arm();
  void updateJointAngles();


  Vec2d m_joints[4];
  int m_numJoints;
  Vec2d m_angleVectors[3]; //Used to get rotations needed. These vectors point in the 0 angle of each bone
  int m_numAngleVectors;
  float m_boneLengths[3]; //Base at start
  int m_numBones;
  Vec2d m_rootLocation;

  float m_jointAngles[3];
  int m_numJointAngles;

  float m_drawThickness;
  
};

class IKSolver {
public:
  IKSolver();

  void solve(Arm& arm, Vec2d target);

private:
  float m_tolerance;
  int m_maxLoops;
};



Vec2d::Vec2d() {
  x = 0;
  y = 0;
}

Vec2d::Vec2d(float ix, float iy) {
  x = ix;
  y = iy;
}

float Vec2d::dot(const Vec2d& other) {
  return x * other.x + y * other.y;
}

bool Vec2d::isCWRotation(const Vec2d& v) {
  float z_comp_cross = v.x * y - x * v.y;
  return z_comp_cross < 0;
}

float Vec2d::magnitude() {
  return pow(pow(x, 2) + pow(y, 2), 0.5);
}

Vec2d operator+(const Vec2d& v1, const Vec2d& v2) {
  return Vec2d(v1.x + v2.x, v1.y + v2.y);
}

Vec2d operator-(const Vec2d& v1, const Vec2d& v2) {
  return Vec2d(v1.x - v2.x, v1.y - v2.y);
}

Vec2d operator*(const float& scalar, const Vec2d& v) {
  return Vec2d(v.x * scalar, v.y * scalar);
}

Vec2d operator*(const Vec2d& v, const float& scalar) {
  return scalar * v;
}


float getSeparation(Vec2d v1, Vec2d v2) {
  return pow(pow(v1.x - v2.x, 2) + pow(v1.y - v2.y, 2), 0.5);
}

Arm::Arm() {
  m_rootLocation = Vec2d(0, 0);
  m_boneLengths[0] = 80;
  m_boneLengths[1] = 80;
  m_boneLengths[2] = 80;
  m_numBones = 3;
  m_joints[0] = m_rootLocation; //Root joint
  m_joints[1] = Vec2d(0, m_boneLengths[0]) + m_rootLocation;
  m_joints[2] = Vec2d(0, m_boneLengths[1]) + m_joints[1];
  m_joints[3] = Vec2d(0, m_boneLengths[2]) + m_joints[2];
  m_numJoints = 4;

  m_angleVectors[0] = Vec2d(0.5736, 0.8192);
  m_angleVectors[1] = Vec2d(0.5735, 0.8196);
  m_angleVectors[2] = Vec2d(0.5735, 0.8196);
  m_numAngleVectors = 3;

  m_drawThickness = 5;

  m_numJointAngles = 3;
}

void Arm::updateJointAngles() {
  for (int i = 0; i < m_numJoints-1; i++) {
    Vec2d compareVector(0, 1);
    if (i > 0) {
      compareVector = m_joints[i] - m_joints[i - 1];
    }
    Vec2d jointVector = m_joints[i + 1] - m_joints[i];
    float angle = acosf(compareVector.dot(jointVector) / (compareVector.magnitude() * jointVector.magnitude()));
    if (!compareVector.isCWRotation(jointVector)) {
      angle = -angle;
    }
    m_jointAngles[i] = angle;

  }


  m_jointAngles[0] *= -1.f;
  //m_jointAngles[1] *= -1.f;
  m_jointAngles[2] *= -1.f;
  //Convert to degrees and center on 90
  for (int i = 0; i < m_numJointAngles; ++i) {
    m_jointAngles[i] *= 57.3;
    m_jointAngles[i] += 90;
  }

  //Buffer
  for (int i = 0; i < m_numJointAngles; ++i) {
    if (m_jointAngles[i] > 175) {
      m_jointAngles[i] = 175;
    }
    if (m_jointAngles[i] < 5) {
      m_jointAngles[i] = 5;
    }
  }


}

IKSolver::IKSolver() {
  m_tolerance = 0.01;
  m_maxLoops = 10;
}

void IKSolver::solve(Arm& arm, Vec2d target) {
  float disToTarget = getSeparation(arm.m_joints[0], target);
  float totalArmLength = 0;

  for (int i = 0; i < arm.m_numBones; ++i) {
    totalArmLength += arm.m_boneLengths[i];
  }

  if (disToTarget > totalArmLength) {
    //std::cout << "IKSolver::solve Distance to target is greater that arm reach" << std::endl;
  }
  else {
    Vec2d initialRootPos = arm.m_joints[0];

    float armEndToTargetDis = getSeparation(arm.m_joints[arm.m_numJoints - 1], target);
    Vec2d newJoints[4];
    for (int i = 0; i < 4; ++i) {
      newJoints[i] = arm.m_joints[i];
    }

    int loopCounter = 0;
    while (armEndToTargetDis > m_tolerance) {
      
      newJoints[3] = target;

      for (int i = 2; i >= 0; i--) {
        float disToNextJoint = getSeparation(newJoints[i + 1], newJoints[i]);
        float lambda = arm.m_boneLengths[i] / disToNextJoint;
        Vec2d newJointPos = (1 - lambda) * newJoints[i + 1] + lambda * newJoints[i];

        //2 doesn't have to undergo joint restraints
        if (i <= 1) {
          //Check angle
          Vec2d setJoint = newJoints[i + 2] - newJoints[i + 1];
          Vec2d prospectiveJoint = newJointPos - newJoints[i + 1];
          float angle = setJoint.dot(prospectiveJoint) / (setJoint.magnitude() * prospectiveJoint.magnitude()); //can be optimized
          if (angle < 0) {
            //good
          }
          else {
            //bad needs moving
            Vec2d perpVector(-setJoint.y, setJoint.x);
            if (perpVector.dot(prospectiveJoint) < 0) {
              perpVector = perpVector * -1.f;
            }
            perpVector = perpVector * (1.f / perpVector.magnitude());
            newJointPos = newJoints[i + 1] + arm.m_boneLengths[i] * perpVector;

          }
        }

        newJoints[i] = newJointPos;
      }

      newJoints[0] = initialRootPos;
      for (int i = 0; i < 3; ++i) {
        float disToNextJoint = getSeparation(newJoints[i], newJoints[i + 1]);
        float lambda = arm.m_boneLengths[i] / disToNextJoint;
        Vec2d newJointPos = (1 - lambda) * newJoints[i] + lambda * newJoints[i + 1];

        if (i >= 1) {
          Vec2d setJoint = newJoints[i - 1] - newJoints[i];
          Vec2d prospectiveJoint = newJointPos - newJoints[i];
          float angle = setJoint.dot(prospectiveJoint) / (setJoint.magnitude() * prospectiveJoint.magnitude());
          if (angle < 0) {

          }
          else {
            Vec2d perpVector(-setJoint.y, setJoint.x);
            if (perpVector.dot(prospectiveJoint) < 0) {
              perpVector = perpVector * -1.f;
            }
            perpVector = perpVector * (1.f / perpVector.magnitude());
            newJointPos = newJoints[i] + arm.m_boneLengths[i] * perpVector;
          }
        }



        newJoints[i + 1] = newJointPos;
      }
      
      for (int i = 0; i < 4; ++i) {
        arm.m_joints[i] = newJoints[i];
      }
      armEndToTargetDis = getSeparation(arm.m_joints[arm.m_numJoints - 1], target);
      
      loopCounter++;
      if (loopCounter > m_maxLoops) {
        break;
        //std::cout << "unable to solve" << std::endl;
      }
    }
  }
}


Arm arm;
IKSolver ikSolver;

Servo BaseMotor;
Servo ShoulderMotor;
Servo ElbowMotor;
Servo WristMotor;

const int BaseEquilAngle = 90;
const int ShoulderEquilAngle = 90; 
const int ElbowEquilAngle = 90; 
const int WristEquilAngle = 90;

const int BaseOffsetAngle = -5;
const int ShoulderOffsetAngle = 5;
const int ElbowOffsetAngle = 5;
const int WristOffsetAngle = 0;

const int sg90PulseLower = 700;
const int sg90PulseHigher = 2300;
const int s3003PulseLower = 400;
const int s3003PulseHigher = 3100;

float targetPos3D[3];

float gain;

void servoWrite(Servo& servo, int angle) {
  servo.write(clip(angle, 0, 180));
}



void setup() {
  // put your setup code here, to run once:
  
  Serial.begin(9600);
  

  pinMode(2, OUTPUT);
  pinMode(3, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(8, INPUT);
  pinMode(9, INPUT);
  pinMode(10, INPUT);

  
  BaseMotor.attach(2, sg90PulseLower, sg90PulseHigher);
  servoWrite(BaseMotor, BaseEquilAngle + BaseOffsetAngle);
  ShoulderMotor.attach(3, s3003PulseLower, s3003PulseHigher);
  servoWrite(ShoulderMotor, ShoulderEquilAngle + ShoulderOffsetAngle);
  ElbowMotor.attach(4, sg90PulseLower, sg90PulseHigher);
  servoWrite(ElbowMotor, ElbowEquilAngle + ElbowOffsetAngle);
  WristMotor.attach(5, sg90PulseLower, sg90PulseHigher);
  servoWrite(WristMotor, WristEquilAngle + WristOffsetAngle);

  targetPos3D[0] = 50; //Along one of star dir, along axis of base motor
  targetPos3D[1] = 100; //Along other star axis
  targetPos3D[2] = 50; // Vertical

  gain = 0.4;

  
}

void loop() {


  float sineCounter = 0;
  float sineAmplitude = 50;
  
  while(true) { 
    
    targetPos3D[0] = 0;
    targetPos3D[1] = 130 + sineAmplitude * sin(sineCounter); 
    targetPos3D[2] = 25; 
    float theta = atanf(targetPos3D[0] / targetPos3D[1]);
    theta = theta * 57.3;
    theta = theta + 90;
    Vec2d targetPos(pow(pow(targetPos3D[0], 2) + pow(targetPos3D[1], 2), 0.5), targetPos3D[2]);
    Serial.println(theta);
    sineCounter += 0.01;
    ikSolver.solve(arm, targetPos);
    arm.updateJointAngles();

    //BaseMotor.write(theta);
    //ShoulderMotor.write(arm.m_jointAngles[0]);
    servoWrite(ShoulderMotor, arm.m_jointAngles[0] + ShoulderOffsetAngle);
    //servoWrite(ElbowMotor, arm.m_jointAngles[1] + ElbowOffsetAngle);
    //servoWrite(WristMotor, arm.m_jointAngles[2] + WristOffsetAngle);
    //ElbowMotor.write(arm.m_jointAngles[1]);
    //WristMotor.write(arm.m_jointAngles[2]);

    //testing
    //servoWrite(ElbowMotor, 90);
    //servoWrite(WristMotor, 90);

    
    //Serial.println(arm.m_jointAngles[0]);
    //Serial.println(arm.m_jointAngles[1]);
    //Serial.println(arm.m_jointAngles[2]);
  }

  
}
