//时钟频率16MHz，0.0625us
int incomedate = 0;//读取数据
int flag_dir = 0;//0表示正向
long num_step = 0;//步数
void setup() {
  //指示灯
  pinMode(LED_BUILTIN, OUTPUT);
  //打开串口
  Serial.begin(9600);
  pinMode(11, OUTPUT);//脉冲
  pinMode(9, OUTPUT);//方向+
  pinMode(8, OUTPUT);//方向-
  digitalWrite(11, LOW);
  digitalWrite(9, HIGH);
  digitalWrite(8, LOW);

}

void loop() {
  if (Serial.available() > 0)//串口接收到数据
  {
    incomedate = Serial.read();//获取串口接收到的数据
    switch (incomedate) {
      //发送脉冲
    case 0xBB:
      //Serial.println("send pluse");
      digitalWrite(11, HIGH);
      delay(1);
      digitalWrite(11, LOW);
      delay(1);
      if (flag_dir == 0) {
        num_step=num_step+1;
        digitalWrite(9, HIGH);
        digitalWrite(8, LOW);
      }
      else if (flag_dir == 1) {
        num_step=num_step-1;
        digitalWrite(9, LOW);
        digitalWrite(8, HIGH);
      }
      break;
    //改变方向
    case 0xAD:
      if (flag_dir == 0) {
        digitalWrite(9, LOW);
        digitalWrite(8, HIGH);
        flag_dir = 1;
      }
      else if (flag_dir == 1) {
        digitalWrite(9, HIGH);
        digitalWrite(8, LOW);
        flag_dir = 0;
      }
      //Serial.print(0x88);//发送的是一个数，得到十进制136
      break;
    case 0xDD:
      Serial.print("A");
      Serial.print(num_step);
      Serial.print("B");
      Serial.print(flag_dir);
      Serial.print("C");
      break;
    }
  }
}
