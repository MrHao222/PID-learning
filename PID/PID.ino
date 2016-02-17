typedef unsigned char      uChar8;
typedef unsigned int       uInt16;
typedef unsigned long int  uInt32;

unsigned char Serial_temp[3];
typedef struct PID_Value
{
  uInt32 liEkVal[3];          //差值保存，给定和反馈的差值
  uChar8 uEkFlag[3];          //符号，1则对应的为负数，0为对应的为正数
  uChar8 uKP_Coe;             //比例系数
  uChar8 uKI_Coe;             //积分常数
  uChar8 uKD_Coe;             //微分常数
  uInt16 iPriVal;             //上一时刻值
  uInt16 iSetVal;             //设定值
  uInt16 iCurVal;             //实际值
} PID_ValueStr;

PID_ValueStr PID;               //定义一个结构体，这个结构体用来存算法中要用到的各种数据
uChar8 i = 0;

void PID_Operation(void)
{
  uInt32 Temp[3] = {0};   //中间临时变量
  uInt32 PostSum = 0;     //正数和
  uInt32 NegSum = 0;      //负数和
  if (PID.iSetVal > PID.iCurVal)               //设定值大于实际值否？
  {
    if (PID.iSetVal - PID.iCurVal > 10)     //偏差大于10否？
      PID.iPriVal = 100;                  //偏差大于10为上限幅值输出(全速加热)
    else                                    //否则慢慢来
    {
      Temp[0] = PID.iSetVal - PID.iCurVal;    //偏差<=10,计算E(k)
      PID.uEkFlag[1] = 0;                     //E(k)为正数,因为设定值大于实际值
      /* 数值进行移位，注意顺序，否则会覆盖掉前面的数值 */
      PID.liEkVal[2] = PID.liEkVal[1];
      PID.liEkVal[1] = PID.liEkVal[0];
      PID.liEkVal[0] = Temp[0];
      /* =================================================================== */
      if (PID.liEkVal[0] > PID.liEkVal[1])             //E(k)>E(k-1)否？
      {
        Temp[0] = PID.liEkVal[0] - PID.liEkVal[1];  //E(k)>E(k-1)
        PID.uEkFlag[0] = 0;                         //E(k)-E(k-1)为正数
      }
      else
      {
        Temp[0] = PID.liEkVal[1] - PID.liEkVal[0];  //E(k)<E(k-1)
        PID.uEkFlag[0] = 1;                         //E(k)-E(k-1)为负数
      }
      /* =================================================================== */
      Temp[2] = PID.liEkVal[1] * 2;                   //2E(k-1)
      if ((PID.liEkVal[0] + PID.liEkVal[2]) > Temp[2]) //E(k-2)+E(k)>2E(k-1)否？
      {
        Temp[2] = (PID.liEkVal[0] + PID.liEkVal[2]) - Temp[2];
        PID.uEkFlag[2] = 0;                         //E(k-2)+E(k)-2E(k-1)为正数
      }
      else                                            //E(k-2)+E(k)<2E(k-1)
      {
        Temp[2] = Temp[2] - (PID.liEkVal[0] + PID.liEkVal[2]);
        PID.uEkFlag[2] = 1;                         //E(k-2)+E(k)-2E(k-1)为负数
      }
      /* =================================================================== */
      Temp[0] = (uInt32)PID.uKP_Coe * Temp[0];        //KP*[E(k)-E(k-1)]
      Temp[1] = (uInt32)PID.uKI_Coe * PID.liEkVal[0]; //KI*E(k)
      Temp[2] = (uInt32)PID.uKD_Coe * Temp[2];        //KD*[E(k-2)+E(k)-2E(k-1)]
      /* 以下部分代码是讲所有的正数项叠加，负数项叠加 */
      /* ========= 计算KP*[E(k)-E(k-1)]的值 ========= */
      if (PID.uEkFlag[0] == 0)
        PostSum += Temp[0];                         //正数和
      else
        NegSum += Temp[0];                          //负数和
      /* ========= 计算KI*E(k)的值 ========= */
      if (PID.uEkFlag[1] == 0)
        PostSum += Temp[1];                         //正数和
      else
        ;   /* 空操作。就是因为PID.iSetVal > PID.iCurVal（即E(K)>0）才进入if的，
                    那么就没可能为负，所以打个转回去就是了 */
      /* ========= 计算KD*[E(k-2)+E(k)-2E(k-1)]的值 ========= */
      if (PID.uEkFlag[2] == 0)
        PostSum += Temp[2];             //正数和
      else
        NegSum += Temp[2];              //负数和
      /* ========= 计算U(k) ========= */
      PostSum += (uInt32)PID.iPriVal;
      if (PostSum > NegSum)                //是否控制量为正数
      {
        Temp[0] = PostSum - NegSum;
        if (Temp[0] < 100 )              //小于上限幅值则为计算值输出
          PID.iPriVal = (uInt16)Temp[0];
        else PID.iPriVal = 100;         //否则为上限幅值输出
      }
      else                                //控制量输出为负数，则输出0(下限幅值输出)
        PID.iPriVal = 0;
    }
  }
  else PID.iPriVal = 0;                       //同上，嘿嘿
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial.println("init begin");
  pinMode(13, OUTPUT);
  Serial.println("init finish");
}

void loop() {
  PID.iSetVal = 200;
  PID.uKP_Coe = 5;
  PID.uKI_Coe = 1;
  PID.uKD_Coe = 1;
  while (1)
  { // put your main code here, to run repeatedly:
    unsigned char temp, no_data;
    if (Serial.available())
    {
      temp = Serial.read();
      no_data = 0;
      if (temp != '\n')
      {
        Serial_temp[i] = temp - 48;
        Serial.println(i);
        Serial.println(Serial_temp[i]);
        i++;
      }
      else
      {
        delayMicroseconds(100);
      }
      if (i > 0 && (temp == '\n'))
      {
        i = 0;
        PID.iCurVal = Serial_temp[0] * 100 + Serial_temp[1] * 10 + Serial_temp[2];
        PID_Operation();
        Serial.print("cuuurent value:"); Serial.println(PID.iCurVal);
        Serial.print("set value:"); Serial.println(PID.iSetVal);
        Serial.print("PID value:"); Serial.println(PID.iPriVal);
        //PID.iCurVal = 0;
        for (i = 0; i < 3; i++)
        {
          Serial_temp[i] = 0;
        }
        i = 0;
      }
    }
  }
}
