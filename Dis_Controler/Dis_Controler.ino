//Filename : Controler.ino
//Discription : 基于Arduino UNO的PWM及PID距离保持控制器

// 引脚定义
const int trigPin = 12;
const int echoPin = 13;

const int PWM_MotorLeft_1 = 5;//定义电机连接引脚
const int PWM_MotorLeft_2 = 6;
const int PWM_MotorRight_1 = 9;
const int PWM_MotorRight_2 = 10;

double distance = 0;

double setpoint = 30;

int WaveBit_Input;
int WaveBit_Output;

int duty;

typedef struct
{
    float Target_value;//目标值
    float Current_value;//当前值
    float PWM;//PWM
    float Current_Error;//目前误差
    float Last_Error;//上一次误差
    float Kp,Ki,Kd;//比例常数，积分常数，微分常数
    float PID_Output;//PID 输出，通常作为控制执行器的变量
    float Integral_value; //积分值
}
PID_TypeDef;
PID_TypeDef PID;//定义结构体变量

void setup()
{
    Serial.begin(9600);
    
    pinMode(trigPin, OUTPUT);//设置Trig引脚
    pinMode(echoPin, INPUT);//设置Echo引脚

    pinMode(PWM_MotorLeft_1, OUTPUT);
    pinMode(PWM_MotorLeft_2, OUTPUT);
    pinMode(PWM_MotorRight_1, OUTPUT);
    pinMode(PWM_MotorRight_2, OUTPUT);
}

//初始化 PID 各项参数
void PID_Init()
{
    PID.Target_value = 0.0;
    PID.Current_value = 0.0;
    PID.PWM = 0.0;
    PID.Current_Error = 0.0;
    PID.Last_Error = 0.0;
    PID.PID_Output = 0.0;
    PID.Integral_value = 0.0;
    PID.Kp = 750.0;
    PID.Ki = 2.5;
    PID.Kd = 0.0;
}

//PID 操作
float PID_operation(float value)
{
    PID.Target_value = value;//确定目标值
    PID.Current_Error = PID.Target_value - PID.Current_value;//计算偏差量
    PID.Integral_value += PID.Current_Error;//计算和
    PID.PID_Output = PID.Kp * PID.Current_Error + PID.Ki * PID.Integral_value + PID.Kd * (PID.Current_Error - PID.Last_Error);//PID运算
    PID.Last_Error = PID.Current_Error;//将当前误差值存入上一误差值
    PID.PWM = PID.PID_Output;//将当前值更新为 PID 的输出值
    PID.Current_value = PID.Current_value + PID.PWM;
    return PID.PWM;//返回当前值
}

//PWM波生产
void PWM(float c)
{
    if(c >= 0)
    {
        if(c >= 1024)
        {
            WaveBit_Input = 1023;
        }
        else if(c < 1024 && c >= 512)
        {
            WaveBit_Input = 511;
        }
        else
        {
            WaveBit_Input = c;
        }
        WaveBit_Output = map(WaveBit_Input, 0, 1023, 0, 255);
        analogWrite(PWM_MotorLeft_1,WaveBit_Output);
        analogWrite(PWM_MotorRight_1,WaveBit_Output);
    }
    else
    {
        if(c <= -1024)
        {
            WaveBit_Input = 1023;
        }
        else if(c > -1024 && c <= -512)
        {
            WaveBit_Input = 511;
        }
        else
        {
            WaveBit_Input = -c;
        }
        WaveBit_Output = map(WaveBit_Input,  0, 1023, 0, 255);
        analogWrite(PWM_MotorLeft_2,WaveBit_Output);
        analogWrite(PWM_MotorRight_2,WaveBit_Output);
    }
}

//获取距离
double Get_Distance()
{
    digitalWrite(trigPin, LOW);//确保Trig引脚处于低电平
    delayMicroseconds(2);//发送10us的高电平脉冲信号
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
      
    distance = pulseIn(echoPin, HIGH);//读取Echo引脚高电平时间, 返回超声波传播的时间，以微妙为单位

    distance = distance/58;//计算距离，声音速度要换算成微妙计算
    
    return distance;
}

void loop()
{
    PID_Init(); //PID 初始化 

    while(1)
    {
        distance = Get_Distance();

        Serial.print("");
        Serial.println(setpoint);
        Serial.print("");
        Serial.println(distance);

        PID.Current_value = distance;
        
        duty = PID_operation(setpoint);
        
        Serial.println(duty);
        
        PWM(duty);
    }   
}
