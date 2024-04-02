/** DO NOT CHANGE THE FOLLOWING DEFINITONS - From UKMARS MazeRunner GitHub **/
/** =============================================================================================================== **/
#if defined(__AVR_ATmega328__) || defined(__AVR_ATmega328P__)
#define __digitalPinToPortReg(P) (((P) <= 7) ? &PORTD : (((P) >= 8 && (P) <= 13) ? &PORTB : &PORTC))
#define __digitalPinToDDRReg(P) (((P) <= 7) ? &DDRD : (((P) >= 8 && (P) <= 13) ? &DDRB : &DDRC))
#define __digitalPinToPINReg(P) (((P) <= 7) ? &PIND : (((P) >= 8 && (P) <= 13) ? &PINB : &PINC))
#define __digitalPinToBit(P) (((P) <= 7) ? (P) : (((P) >= 8 && (P) <= 13) ? (P)-8 : (P)-14))

// general macros/defines
#if !defined(BIT_READ)
#define BIT_READ(value, bit) ((value) & (1UL << (bit)))
#endif
#if !defined(BIT_SET)
#define BIT_SET(value, bit) ((value) |= (1UL << (bit)))
#endif
#if !defined(BIT_CLEAR)
#define BIT_CLEAR(value, bit) ((value) &= ~(1UL << (bit)))
#endif
#if !defined(BIT_WRITE)
#define BIT_WRITE(value, bit, bitvalue) (bitvalue ? BIT_SET(value, bit) : BIT_CLEAR(value, bit))
#endif

#define fast_write_pin(P, V) BIT_WRITE(*__digitalPinToPortReg(P), __digitalPinToBit(P), (V));
#define fast_read_pin(P) ((int)(((BIT_READ(*__digitalPinToPINReg(P), __digitalPinToBit(P))) ? HIGH : LOW)))

#else
#define fast_write_pin(P, V) fast_write_pin(P, V)
#define fast_read_pin(P) digitalRead(P)
#endif

/** =============================================================================================================== **/

/** DEFINE OUR PINS AND WHICH COMPONENTS THEY ARE CONNECTED TO **/
/** _______________________________________________________________________________________________________________ **/
const int ENCODER_R_A = 3; // ENCODER RIGHT A (ticks first when motor forward)
const int ENCODER_R_B = 5; // ENCODER RIGHT B (ticks first when motor backward) 

const int ENCODER_L_A = 4; // ENCODER LEFT A (ticks first when motor forward)
const int ENCODER_L_B = 2; // ENCODER LEFT B (ticks first when motor backward)

const int SPEED_MOTOR_L = 9; // PWM MOTOR LEFT 
const int SPEED_MOTOR_R = 10; // PWM MOTOR RIGHT 

const int DIR_MOTOR_L = 7; // DIRECTION MOTOR LEFT 
const int DIR_MOTOR_R = 8; // DIRECTION MOTOR RIGHT 

#define L analogRead(A2)
#define M analogRead(A1)
#define R analogRead(A0)

// 4 Way switch and push button
const int DIP_SWITCH = A6; 
/** _______________________________________________________________________________________________________________ **/

/* GLOBAL VARIABLES */

volatile int rightEncoderPos = 0; // Counts for right encoder ticks
volatile int leftEncoderPos = 0; // Counts for left encoder ticks

// Variables to help us with our PID
int prevTime_r = 0;
int prevError_r;
int errorIntegral_r;
int prevTime_l = 0;
int prevError_l;
int errorIntegral_l;
bool switchOn;

void setup() {
  Serial.begin(9600);
  pinMode (12, OUTPUT);
  digitalWrite(12, HIGH);
  pinMode(ENCODER_R_A, INPUT_PULLUP);
  pinMode(ENCODER_R_B, INPUT_PULLUP);
  pinMode(ENCODER_L_A, INPUT_PULLUP);
  pinMode(ENCODER_L_B, INPUT_PULLUP);

  pinMode(SPEED_MOTOR_L, OUTPUT);
  pinMode(SPEED_MOTOR_R, OUTPUT);
  pinMode(DIR_MOTOR_L, OUTPUT);
  pinMode(DIR_MOTOR_R, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(ENCODER_L_B), readEncoderLeft, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_R_A), readEncoderRight, CHANGE);
}

/** INTERRUPT SERVICE ROUTINES FOR HANDLING ENCODER COUNTING USING STATE TABLE METHOD **/
void readEncoderLeft() {
  static uint8_t prevState = 0; 
  static uint8_t currState = 0; 
  static unsigned long lastTime = 0; 
  
  currState = (fast_read_pin(ENCODER_L_B) << 1) | fast_read_pin(ENCODER_L_A);
  
  unsigned long currentTime = micros();
  unsigned long deltaTime = currentTime - lastTime;
  lastTime = currentTime;
  
  // direction based on prev state
  uint8_t direction = (prevState << 2) | currState;
  switch(direction) {
    case 0b0001:
    case 0b0111:
    case 0b1110:
    case 0b1000:
      leftEncoderPos++;
      break;
    case 0b0010:
    case 0b1100:
    case 0b0101:
    case 0b1011:
      leftEncoderPos--;
      break;

    default:
      break;
  }

  prevState = currState;
}
void readEncoderRight() {
  static uint8_t prevState = 0; 
  static uint8_t currState = 0; 
  static unsigned long lastTime = 0; 
  
  currState = (fast_read_pin(ENCODER_R_B) << 1) | fast_read_pin(ENCODER_R_A);
  
  unsigned long currentTime = micros();
  unsigned long deltaTime = currentTime - lastTime;
  lastTime = currentTime;
  
  uint8_t direction = (prevState << 2) | currState;
  switch(direction) {
    case 0b0100:
    case 0b1010:
    case 0b0111:
    case 0b1001:
      rightEncoderPos++;
      break;
    case 0b1000:
    case 0b0110:
    case 0b1101:
    case 0b0011:
      rightEncoderPos--;
      break;

    default:
      break;
  }
  
  prevState = currState;
}

/** Function to set motor speed and direction for BOTH motors
    @params dir - can either be HIGH or LOW for clockwise / counter clockwise rotation
    @params speed - analogWrite() value between 0-255
**/
void setMotors_R(int dir, int speed){
  analogWrite(SPEED_MOTOR_R, speed);
  
  if(dir == 1){
    fast_write_pin(DIR_MOTOR_R, HIGH);
  } else if (dir == -1){
    fast_write_pin(DIR_MOTOR_R, LOW);
  } else{
    analogWrite(SPEED_MOTOR_R, 0);
  }
}

void setMotors_L(int dir, int speed){
  analogWrite(SPEED_MOTOR_L, speed);
  
  if(dir == 1){
    fast_write_pin(DIR_MOTOR_L, HIGH);
  } else if (dir == -1){
    fast_write_pin(DIR_MOTOR_L, LOW);
  } else{
    analogWrite(SPEED_MOTOR_L, 0);
  }
}

/** Function to make the robot travel for a certain amount of encoder ticks, calls upon setMotors at end
    @params dir - setPoint: The target value for how far we want to go (in encoder ticks)
    @params speed - analogWrite() value between 0-255
    @params kp - proportional gain, this is the main one you should be changing
    @params ki - intergral gain, use this for steady state errors
    @params kd - derivative gain, use this for overshoot and oscillation handling 
**/
void motorPID_R(int setPoint, float kp, float ki, float kd){
  int currentTime = micros();
  int deltaT = ((float)(currentTime - prevTime_r)) / 1.0e6; // time difference between ticks in seconds
  prevTime_r = currentTime; // update prevTime each loop 
  
  int error = setPoint - rightEncoderPos;
  int errorDerivative = (error - prevError_r) / deltaT;
  errorIntegral_r = errorIntegral_r + error*deltaT;

  float u = kp*error + ki*errorIntegral_r + kd*errorDerivative; 

  float speed = fabs(u);
  if(speed > 255){
    speed = 255;
  }

  int dir = 1;
  if (u < 0) {
    dir = -1; // Move backward
  } else {
    dir = 1; // Move forward
  }

  setMotors_R(dir, speed);
  prevError_r = 0;
}

void motorPID_L(int setPoint, float kp, float ki, float kd){
  int currentTime = micros();
  int deltaT = ((float)(currentTime - prevTime_l)) / 1.0e6; // time difference between ticks in seconds
  prevTime_l = currentTime; // update prevTime each loop 
  
  int error = setPoint - leftEncoderPos;
  int errorDerivative = (error - prevError_l) / deltaT;
  errorIntegral_l = errorIntegral_l + error*deltaT;

  float u = kp*error + ki*errorIntegral_l + kd*errorDerivative; 

  float speed = fabs(u);
  if(speed > 255){
    speed = 255;
  }

  int dir = 1;
  if (u < 0) {
    dir = -1; // Move backward
  } else {
    dir = 1; // Move forward
  }

  setMotors_L(dir, speed);
  prevError_l = 0;
}
//==============================================================================================
// YOUR HOMEWORK ASSIGNMENT: Create a function to convert from encoder ticks to centimeters!
int tickConvertToCm(int encoderTicks)
{
  return (encoderTicks/3)*0.016;
}
//==============================================================================================

void turn(int direction)
{
  analogWrite(SPEED_MOTOR_L, 255);
  analogWrite(SPEED_MOTOR_R, 255);
  switch (direction)
  {
    case -1:
      fast_write_pin(DIR_MOTOR_L, HIGH);
      fast_write_pin(DIR_MOTOR_R, LOW);
      break;
    case 1:
      fast_write_pin(DIR_MOTOR_L, LOW);
      fast_write_pin(DIR_MOTOR_R, HIGH);
      break;
  }
}

/** CLASSES **/

struct Coordinate
{
  public:
    int x, y;
    Coordinate(int x, int y)
    {
      this -> x = x;
      this -> y = y;
    }
    Coordinate add(Coordinate input)
    {
      x += input.x;
      y += input.y;
    }
    Coordinate reverse()
    {
      return Coordinate(y, x);
    }
    Coordinate negate()
    {
      return Coordinate(-x, -y);
    }
    Coordinate left()
    {
      return (y == 0)?(reverse()):(reverse().negate());
    }
    Coordinate right()
    {
      return(y == 0)?(reverse().negate()):(reverse());
    }
    bool within(int xMin, int xMax, int yMin, int yMax)
    {
      return (x < xMax && x >= xMin && y < yMax && y >= yMin);
    }
};

class Node
{
  private:
    Node* accessibleNodes[4];
    int weight;
    int head;
    int tail;
  public:
    Node()
    {
      head = 0;
      tail = 0;
      weight = -1;
    }
    int getWeight()
    {
      return weight;
    }
    void setWeight(int input)
    {
      weight = input;
    }
    void setAccessibleNode(Node* input)
    {
      accessibleNodes[tail] = input;
      tail++;
    };
    int getTail()
    {
      return tail;
    }
    Node* getNode()
    {
      Node* output = accessibleNodes[head];
      (head <= tail)?(head++):(head = 0);
      return output;
    }
    void removeNode(Node* input)
    {
      if (input != NULL)
      {
        int position = -1;
        for (int i = 0; i <= tail; i++)
        {
          if (accessibleNodes[i] == input)
          {
            accessibleNodes[i] = NULL;
            position = i;
            break;
          }
          if (position != tail || position > -1)
          {
            for (int i = position; i < tail; tail++)
            {
                accessibleNodes[i] = accessibleNodes[i + 1];
                accessibleNodes[i + 1] = NULL;
            }
          }
          tail--;
        }
      }
    }

    void flood()
    {
      weight = weight + 1;
      for (int i = 0; i <= tail; i++)
      {
        getNode() -> Node::flood(weight);
      }
    }
    void flood(int previousWeight)
    {
      weight = previousWeight + 1;
      for (int i = 0; i <= tail; i++)
      {
        Node* currentNode = getNode();
        if (currentNode -> Node::getWeight() < 0 || currentNode -> Node::getWeight() > weight)
        {
          currentNode -> Node::flood(weight);
        }
      }
    }
};

class Graph
{
  private:
      Node grid[8][8];
      Node* zeroNode;

      void clear()
      {
        for (int i = 0; i < 8; i++)
        {
          for (int j = 0; j < 8; j++)
          {
            grid[i][j].setWeight(-1);
          }
        }
      }
    public:
      Graph()
      {
        for (int x = 0; x < 8; x++)
        {
          for (int y = 0; y < 8; y++)
          {
            grid[x][y] = Node();
          }
        }
        for (int i = 0; i < 8; i++)
        {
          for (int j = 0; j < 8; j++)
          {
            if (i > 0)
            {
              grid[i][j].setAccessibleNode(&grid[i-1][j]);
            }
            if (j > 0)
            {
              grid[i][j].setAccessibleNode(&grid[i][j-1]);
            }
            if (i < 7)
            {
              grid[i][j].setAccessibleNode(&grid[i+1][j]);
            }
            if (j < 7)
            {
              grid[i][j].setAccessibleNode(&grid[i][j+1]);
            }
          }
        }
        zeroNode = &grid[4][4];
      };

      Node* getNode(Coordinate input)
      {
        return (input.within(0, 8, 0, 8))?(&grid[input.x][input.y]):(NULL);
      }

      void createWall(Coordinate currentPosition, Coordinate direction)
      {
        if (M > 125)
        {
          getNode(currentPosition) -> Node::removeNode(getNode(currentPosition.add(direction)));
          getNode(currentPosition.add(direction)) -> Node::removeNode(getNode(currentPosition));
        }
        if (L > 125)
        {
          getNode(currentPosition) -> Node::removeNode(getNode(currentPosition.add(direction.left())));
          getNode(currentPosition.add(direction.left())) -> Node::removeNode(getNode(currentPosition));
        }
        if (R > 125)
        {
          getNode(currentPosition) -> Node::removeNode(getNode(currentPosition.add(direction.right())));
          getNode(currentPosition.add(direction.right())) -> Node::removeNode(getNode(currentPosition));
        }
      }

      void flood()
      {
        zeroNode -> Node::flood();
      }
};

/// direction, position and maze graph
Graph graph = Graph();
Coordinate direction = Coordinate(1, 0);
Coordinate position = Coordinate(0, 0);


void loop() {
  // Starter Code
  int dipSwitch = analogRead(DIP_SWITCH);
  //Serial.println(dipSwitch);
  if(dipSwitch > 1000){
    switchOn = true;
  }

  //if(switchOn){
    //delay(500); // Wait half a second after pressing the button to actually start moving, safety first!

    //graph.flood();

    int setPoint = 1000;
    float kp = 1;
    float ki = 0.1;
    float kd = 0.001;
    motorPID_R(-setPoint, kp, ki, kd);
    motorPID_L(setPoint, kp, ki, kd);

    //graph.createWall(position, direction);

    Serial.print(setPoint);
    Serial.print(" ");
    Serial.print(rightEncoderPos);
    Serial.println();
  //}
}
