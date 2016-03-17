#include <MsTimer2.h>

//データバッファ
uint8_t bleBuffer[6];

//加速度
AccelerationReading accel = {0, 0, 0};

//タイマ割込発生数カウンタ
uint16_t count = 0;

//割込中フラグ
bool readFlag = false;

//LED点灯
static bool led = true;

// タイマ割り込みによるアナログポート読み込み
void ReadAD()
{
  //割込許可（タイマを止めないため）
  interrupts();
  readFlag = true;
  count++;

  //加速度の読み込み
  accel = Bean.getAcceleration();

  //LED点滅
  if (count % 10 == 0) {
    led = !led;
    if (led) Bean.setLed(32, 0, 0);
    else Bean.setLed(0, 0, 0);
  }
}

void setup()
{
  //LEDをオン
  Bean.setLed(32, 0, 0);

  //1000msごとに割込みA/D読取を起動。
  MsTimer2::set(1000, ReadAD);
  MsTimer2::start();
}

// the loop routine runs over and over again forever:
void loop()
{
  //readフラグ（読み取りデータあり）が立っていればデータ送信
  if (readFlag) {

    //フラグリセット
    readFlag = false;

    //センサ情報をバッファに格納

    int16_t y = (int16_t) Filter(accel.xAxis);

    //バッファカウンタ
    uint16_t bufferCounter = 0;
    bleBuffer[bufferCounter++] = count & 0xFF;
    bleBuffer[bufferCounter++] = count >> 8;
    bleBuffer[bufferCounter++] = y & 0xFF;
    bleBuffer[bufferCounter++] = y >> 8;
    bleBuffer[bufferCounter++] = accel.xAxis & 0xFF;
    bleBuffer[bufferCounter++] = accel.xAxis >> 8;

    //BLE送信
    Bean.setScratchData(1, bleBuffer, sizeof(bleBuffer));

    //BeanLoaderのバーチャルシリアルによる出力
    Serial.print(count, DEC);
    Serial.print("\t");
    Serial.print(y, DEC);
    Serial.print("\t");
    Serial.print(accel.xAxis, DEC);
    Serial.print("\n");
  }
}

// 2次フィルタ（直接形II転置形、Transposed-Direct-Form-II）
// 分子係数 a、分母係数bを初期設定して用いる。
// y = Filter(x);
// x:入力
// y:フィルタ出力
double Filter(double x)
{
  //z変換された伝達関数の分子係数（２次ローパス、無次元カットオフ：0.2（カットオフ0.1Hz））
  static const double a[3] = {0.06745527388907, 0.13491054777814, 0.06745527388907};
  //伝達関数の分母係数
  static const double b[3] = {1.0, -1.14298050253990, 0.41280159809619};
  //遅延バッファ
  static double u[2] = {0, 0};

  //フィルタ出力の計算
  double y  = a[0] * x + u[0];
  u[0] = a[1] * x  - b[1] * y + u[1];
  u[1] = a[2] * x  - b[2] * y;
  return y;
}
