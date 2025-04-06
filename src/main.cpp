#include <Arduino.h>
#include <ESP32Servo.h>
#include <Ticker.h>


Servo servo1;
//舵机控制范围初始化值
#define minUs  400
#define maxUs  2400
//舵机控制引脚
#define servo1Pin 18
//电机PWM控制引脚
#define PWM_MOTOR_Pin 1
//红外接收器状态反馈IO
#define redPin1 0
#define redPin2 2
#define redPin3 3
#define redPin4 5
#define redPin5 19
#define redPin6 7
#define redPin7 6
#define redPin8 10
#define redPin9 4
#define redPin10 8

//速度挡位
#define HIGH_SPEED 255
#define MID_SPEED  180 //70
#define LOW_SPEED  127 //50

// 定义定时器对象
Ticker timer;

//定义代码中使用的数据类型
  int redInfo[10]; //红外接收数组
  int rayTry[10];  //红外数据临时数组
  int flagReceive=1; //红外接收标志位，无信号为1，有信号为0
  int red_recivenum=0;//测试使用
  int myAngle=0;
  int pid_init_flag=0;
  int first_start_flag=0;
  int count_flag=0;
  int time_count_num;

// 结构体声明
typedef struct
{
	//PID 参数系数K
	float Kp;
	float Ki;
	float Kd;
	
	//PID 控制量C:当前: C0  上一次:C1 范围控制量:Cmin Cmax 
	float C0;
	float C1;
	float C2;
	float Cmin;
	float Cmax;
	
	
    //PID 误差量E 当前: E0  上一次: E1 上上一次: E2
	float E0;
	float E1;
	float E2;	
	
	unsigned char  u8PIDInitedFlag;
}PID_TypeDef; //PID 参数


PID_TypeDef sPIDPara;



void infraredReceiver_Init()
{ //红外引脚配置为输入上拉模式
  pinMode(redPin1, INPUT_PULLUP);
  pinMode(redPin2, INPUT_PULLUP);
  pinMode(redPin3, INPUT_PULLUP);
  pinMode(redPin4, INPUT_PULLUP);
  pinMode(redPin5, INPUT_PULLUP);
  pinMode(redPin6, INPUT_PULLUP);
  pinMode(redPin7, INPUT_PULLUP);
  pinMode(redPin8, INPUT_PULLUP);
  pinMode(redPin9, INPUT_PULLUP);
  pinMode(redPin10, INPUT_PULLUP);

}
/*
  *
  * @描述  软件定时器回调函数
  * @参数  当定时器触发中断时，进入该程序，将中断标志位设置为高
  * @返回值 Vaule:当标志位为1时，主函数loop循环中对对应变量进行+1，并清零标志位
  */
 
void count_RED(void)
{
  count_flag=1;//触发时将标志位置高
  time_count_num++;
  
}


/*
  *
  * @描述  红外过滤
  * @参数  Value=1,表示没有收到红外；getInfo:当前红外数组里的值
  * @返回值 Vaule:当前是否收到了红外
  */
int filteRed(int Value, int getInfo)
{
  int divNum = 0;
  int newValue;
  newValue = getInfo;
  while (Value != newValue) //如果收到红外了
  {
    divNum++;
    if (divNum >= 5) return newValue; //持续到5次++后，返回获取的值
     delayMicroseconds(10);
    newValue = getInfo;//把获取的值与value=1 做比较，保持while循环
  }
  return Value;//获取的值为 1时，返回1（未按下）
}


/*
  *
  * @描述  计算出舵机要摆动的角度
  * @参数  int* redInfor 传入存储红外接收状态数组
  * @返回值 angle 角度
  */
float obtainAngle(int *redInfor)
{
  float divNum, sum, angle;
  sum = 0;
  divNum = 1;
  if (redInfo[1] == 0) sum = sum -88, divNum++;
  if (redInfo[2] == 0) sum = sum -65, divNum++;
  if (redInfo[3] == 0) sum = sum -45, divNum++;
  if (redInfo[4] == 0) sum = sum -25, divNum++;
  if (redInfo[5] == 0) sum = sum +10, divNum++;
  if (redInfo[6] == 0) sum = sum +15, divNum++;
  if (redInfo[7] == 0) sum = sum +35, divNum++;
  if (redInfo[8] == 0) sum = sum +55, divNum++;
  if (redInfo[9] == 0) sum = sum +75, divNum++;
  if (redInfo[10] == 0) sum = sum +85, divNum++;
  angle=(int)sum/divNum;
  return angle;
}



void setup() {
    Serial.begin(9600);
    Serial.print("串口初始化成功\n");
    infraredReceiver_Init();
    Serial.print("红外引脚初始化成功\n");
    //频率设置50Hz 1s 50个脉冲 每个脉冲20ms
    servo1.setPeriodHertz(50);      // Standard 50hz servo
    servo1.attach(servo1Pin, minUs, maxUs);//sg90 舵机 400-2400
    Serial.print("舵机初始化成功\n");
    pinMode(PWM_MOTOR_Pin, OUTPUT);
    Serial.print("电机初始化成功\n");
  // 配置周期性定时器
  timer.attach(0.5, count_RED);
 
}


void loop()
{

  
   float Kp=1.5;
   float Ki=0;
   float Kd=0.3;

  if(pid_init_flag==0)
  {
  pid_init_flag=1;//仅在初始化阶段有效，不参与后续循环

  sPIDPara.C0 = 0;
	sPIDPara.C1 = 0;
	sPIDPara.Cmin = -60;
	sPIDPara.Cmax = 60;
	
	sPIDPara.E0 = 0;
	sPIDPara.E1 = 0;
	sPIDPara.E2 = 0;	
	
	sPIDPara.u8PIDInitedFlag=1;
  Serial.print("PID初始化成功\n");
  
  } 



   for (int i = 1; i < 11; i++)
  { //给数组赋初始值
    redInfo[i] = 1;
    rayTry[i] = 1;
  }
    //滤波
    for(int i = 0;i<800;i++)
    {
    rayTry[1] = filteRed(1, digitalRead(redPin1)); //IO2
    rayTry[2] = filteRed(1, digitalRead(redPin2)); //IO3
    rayTry[3] = filteRed(1, digitalRead(redPin3)); //IO4
    rayTry[4] = filteRed(1, digitalRead(redPin4)); //IO5
    rayTry[5] = filteRed(1, digitalRead(redPin5)); //IO6
    rayTry[6] = filteRed(1, digitalRead(redPin6)); //IO7
    rayTry[7] = filteRed(1, digitalRead(redPin7)); //IO8
    rayTry[8] = filteRed(1, digitalRead(redPin8)); //IO9
    rayTry[9] = filteRed(1, digitalRead(redPin9)); //IO10
    rayTry[10] = filteRed(1, digitalRead(redPin10)); //IO12

    redInfo[1] &= rayTry[1];
    redInfo[2] &= rayTry[2];
    redInfo[3] &= rayTry[3];
    redInfo[4] &= rayTry[4];
    redInfo[5] &= rayTry[5];
    redInfo[6] &= rayTry[6];
    redInfo[7] &= rayTry[7];
    redInfo[8] &= rayTry[8];
    redInfo[9] &= rayTry[9];
    redInfo[10] &= rayTry[10];
    delayMicroseconds(100);
    }
  flagReceive = redInfo[1] & redInfo[2] & redInfo[3] & redInfo[4] & redInfo[5] & redInfo[6] & redInfo[7] & redInfo[8] & redInfo[9] & redInfo[10];
  
 if(flagReceive == 0)
 {
  time_count_num=0; //对定时器的值进行清0，防止进入else的自旋寻信号模式
  Serial.print("接收到红外信号\n");
  for (int k = 1; k <= 10; k++)
    { //输出当前数组的值（红外接收情况）
      if( redInfo[k] == 0)
      {
        Serial.print("redInfo");
        Serial.print(k);
        Serial.print(" = ");
        Serial.println(redInfo[k]);
      } 
    } 
  myAngle = obtainAngle(&redInfo[0]);
     
  Serial.print("myAngle:");
  Serial.println(myAngle);
   // Serial.println(myAngle);
  //pid 参数初始化
	
		sPIDPara.E0 = myAngle;
		sPIDPara.C0 = Kp*(sPIDPara.E0-sPIDPara.E1);
                  +Ki*(sPIDPara.E0)
                  +Kd*(sPIDPara.E0-2*sPIDPara.E1+sPIDPara.E2);
		
		sPIDPara.C0 = sPIDPara.C0 + sPIDPara.C1;
		sPIDPara.C1 = sPIDPara.C0;
		
		sPIDPara.E2 = sPIDPara.E1;
		sPIDPara.E1 = sPIDPara.E0;
		
		if(sPIDPara.C0 > sPIDPara.Cmax)
		{
			sPIDPara.C0 = sPIDPara.Cmax;
		}
		
		if(sPIDPara.C0 < sPIDPara.Cmin)
		{
			sPIDPara.C0 = sPIDPara.Cmin;
		}
		
		//以下两句是实际执行舵机
    int PID_Angle=0;
    PID_Angle=sPIDPara.C0+90;
    servo1.write(PID_Angle);
    Serial.print("PID_Angle:");
    Serial.println(PID_Angle);
    //以下实际执行电机
    analogWrite(PWM_MOTOR_Pin, HIGH_SPEED);
 
 }
 else//如果没收到信号
 {
  
  if(flagReceive == 1&& time_count_num == 3)//重复判断是否没有收到红外信号
  {
    time_count_num=0;

    if(first_start_flag==0)
    {
    first_start_flag= 1;
    Serial.println("初始化后没收到红外信号");
    //舵机调正
    servo1.write(90);
    //驱动电机全速直行
    Serial.println("电机全速直行");
    analogWrite(PWM_MOTOR_Pin, HIGH_SPEED);
    //执行1s
    delay(1000);
    }
  else
   {
    time_count_num=0;
    Serial.println("没收到红外信号");
    Serial.println("低速自转中");
    analogWrite(PWM_MOTOR_Pin, LOW_SPEED);
    servo1.write(45);
    }
  }
    
 }
  
}






