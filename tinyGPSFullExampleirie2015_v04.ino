/* 
赤崎水曜日郵便局　灯台ポスト　プログラム　ver 0.03 
H.Irie 
2015/01/13 
水曜日を検出して、午後5時（JST、プログラムではUTCの午前８時）に
リレーをオンして投打ポストの電球を灯すプログラムを作成。
リレーはラッチングリレーを使用、１２をONで　通電。 13オンで遮断
GPSは2,3を使用。　6,７はシリアルLCDで、GPSの時刻と年月日、緯度経度を表示。


2015/02/18 Wed.
4ピンに繋がっているボタンを押し4ピンの電圧がHighになれば、
１０分だけ連続点灯するプログラムを入れた
*/

/* sleepを利用した待ち時間関数のサブルーチンを追加
動作試験中 2015-04-29(wed)  コンパイルして動作を確認したが、
GPSの動作を確認できず。

*/
#include <SerialLCD.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <AltSoftSerial.h>

#include <avr/sleep.h>  //2015-02-28  WDT & sleep を追加
#include <avr/wdt.h>

//点灯時間定義
#define LIGHTUP_HOUR 9  //time use by UTC   8 is 17 in JST. 9 is 18 in JST.  It was changed to 9 at 2015/04/29 by H.Irie
#define LIGHTUP_MIN 0
#define ONDAY_def  3 // 0　is Sunday, therefore 3 is Wednesday,
#define ON_HOUR 5l   //5l is 5 hours  It's lightingn time 

/*点灯式用 2014/12/20 (Sat.)
#define LIGHTUP_HOUR 17
#define LIGHTUP_MIN 18
*/

#define RL_SET 12 // I/O No.12
#define RL_RESET 13//I/O No.13


/*
   This sample code demonstrates the normal use of a TinyGPS++ (TinyGPSPlus) object.
   It requires the use of SoftwareSerial, and assumes that you have a
   4800-baud serial GPS device hooked up on pins 4(rx) and 3(tx).
*/
static const int RXPin = 2, TXPin = 3; //using SoftSerial I/O port for ublox 4H
static const uint32_t GPSBaud = 9600;

// The TinyGPS++ object
TinyGPSPlus gps;

// The serial connection to the GPS device
SoftwareSerial ss(RXPin, TXPin);
//AltSoftSerial ss;

// initialize the library
SerialLCD slcd(6,7);//this is a must, assign soft serial pins


// constants won't change. They're used here to 
// set pin numbers:
const int buttonPin = 4;     // the number of the pushbutton pin


// This custom version of delay() ensures that the gps object
// is being "fed".
static void smartDelay(unsigned long ms)
{
  unsigned long start = millis();
  do 
  {
    while (ss.available())
      gps.encode(ss.read());
  } while (millis() - start < ms);
}

static void printFloat(float val, bool valid, int len, int prec)
{
  if (!valid)
  {
    while (len-- > 1)
      Serial.print('*');
    Serial.print(' ');
  }
  else
  {
    Serial.print(val, prec);
    int vi = abs((int)val);
    int flen = prec + (val < 0.0 ? 2 : 1); // . and -
    flen += vi >= 1000 ? 4 : vi >= 100 ? 3 : vi >= 10 ? 2 : 1;
    for (int i=flen; i<len; ++i)
      Serial.print(' ');
  }
  smartDelay(0);
}

static void SlcdprintFloat(float val, bool valid, int len, int prec)
{
  if (!valid)
  {
    while (len-- > 1)
      slcd.print('*');
    slcd.print(' ');
  }
  else
  {
    int vala,valb;
    
    vala =  (int)val;
    valb=  (int)((val-vala)*10000);
    
    char sz[32];
    sprintf(sz, "%d.%d ", vala,valb);
  //  slcd.print(val,3);// slcd.print('.'); slcd.print(valb);
   slcd.print(sz);
    
  }
  
}

static void printInt(unsigned long val, bool valid, int len)
{
  char sz[32] = "*****************";
  if (valid)
    sprintf(sz, "%ld", val);
  sz[len] = 0;
  for (int i=strlen(sz); i<len; ++i)
    sz[i] = ' ';
  if (len > 0) 
    sz[len-1] = ' ';
  Serial.print(sz);
  smartDelay(0);
}

static void printDateTime(TinyGPSDate &d, TinyGPSTime &t)
{
  if (!d.isValid())
  {
    Serial.print(F("********** "));
  }
  else
  {
    char sz[32];
    sprintf(sz, "%02d/%02d/%02d ", d.month(), d.day(), d.year());
    Serial.print(sz);
  }
  
  if (!t.isValid())
  {
    Serial.print(F("******** "));
  }
  else
  {
    char sz[32];
    sprintf(sz, "%02d:%02d:%02d ", t.hour(), t.minute(), t.second());
    Serial.print(sz);
  }

  printInt(d.age(), d.isValid(), 5);
  smartDelay(0);
}

static void SlcdprintDateTime(TinyGPSDate &d, TinyGPSTime &t)
{
  if (!d.isValid())
  {
    slcd.print("********** ");
  }
  else
  {
    char sz[32];
    int heisei;
    heisei=27+(d.year()-2015);
    sprintf(sz, "%02d/%02d/H%02d ", d.month(), d.day(), heisei);
    slcd.print(sz);
  }
  
  if (!t.isValid())
  {
    slcd.print("******** ");
  }
  else
  {
    char sz[32];
    sprintf(sz, "%02d%02d%02d ", t.hour(), t.minute(), t.second());
    slcd.print(sz);
  }

//  printInt(d.age(), d.isValid(), 5);
}


static void printStr(const char *str, int len)
{
  int slen = strlen(str);
  for (int i=0; i<len; ++i)
    Serial.print(i<slen ? str[i] : ' ');
  smartDelay(0);
}

static void printValue(void)
{
 Serial.println(gps.location.lat()); // Latitude in degrees (double)
Serial.println(gps.location.lng()); // Longitude in degrees (double)
Serial.print(gps.location.rawLat().negative ? "-" : "+");
Serial.println(gps.location.rawLat().deg); // Raw latitude in whole degrees
Serial.println(gps.location.rawLat().billionths);// ... and billionths (u16/u32)
Serial.print(gps.location.rawLng().negative ? "-" : "+");
Serial.println(gps.location.rawLng().deg); // Raw longitude in whole degrees
Serial.println(gps.location.rawLng().billionths);// ... and billionths (u16/u32)
Serial.println(gps.date.value()); // Raw date in DDMMYY format (u32)
Serial.println(gps.date.year()); // Year (2000+) (u16)
Serial.println(gps.date.month()); // Month (1-12) (u8)
Serial.println(gps.date.day()); // Day (1-31) (u8)
Serial.println(gps.time.value()); // Raw time in HHMMSSCC format (u32)
Serial.println(gps.time.hour()); // Hour (0-23) (u8)
Serial.println(gps.time.minute()); // Minute (0-59) (u8)
Serial.println(gps.time.second()); // Second (0-59) (u8)
Serial.println(gps.time.centisecond()); // 100ths of a second (0-99) (u8)
Serial.println(gps.speed.value()); // Raw speed in 100ths of a knot (i32)
Serial.println(gps.speed.knots()); // Speed in knots (double)
Serial.println(gps.speed.mph()); // Speed in miles per hour (double)
Serial.println(gps.speed.mps()); // Speed in meters per second (double)
Serial.println(gps.speed.kmph()); // Speed in kilometers per hour (double)
Serial.println(gps.course.value()); // Raw course in 100ths of a degree (i32)
Serial.println(gps.course.deg()); // Course in degrees (double)
Serial.println(gps.altitude.value()); // Raw altitude in centimeters (i32)
Serial.println(gps.altitude.meters()); // Altitude in meters (double)
Serial.println(gps.altitude.miles()); // Altitude in miles (double)
Serial.println(gps.altitude.kilometers()); // Altitude in kilometers (double)
Serial.println(gps.altitude.feet()); // Altitude in feet (double)
Serial.println(gps.satellites.value()); // Number of satellites in use (u32)
Serial.println(gps.hdop.value()); // Horizontal Dim. of Precision (100ths-i32)
   
 
}

void slcdPrint(){
  ss.end(); //GPSからの読み込みを止める。
 // slcd.begin();

  slcd.setCursor(0, 0);
 
 SlcdprintFloat(gps.location.lat(), gps.location.isValid(), 11, 6);
  // print the number of seconds since reset:
//slcd.println(gps.location.lat(),DEC); // Latitude in degrees (double)
  slcd.setCursor(8, 0);
//slcd.println(gps.location.lng(),DEC); // Longitude in degrees (double)
 SlcdprintFloat(gps.location.lng(), gps.location.isValid(), 12, 6);
  slcd.setCursor(0, 1);
SlcdprintDateTime(gps.date, gps.time);

// slcd.end();
  
  ss.begin(GPSBaud);//GPSからの読み込みを再開する
  
  
}

/*
 　点灯するかどうかを判定するルーテン
　　あらかじめGPSからのデータを取得しておく
*/
void SW_loop()
{
  int ONDAY; 
  static int vackup_time_stamp = 0;
 
  ONDAY = ONDAY_def;
  
  
  if( gps.date.isValid() != 0){
    Serial.println("youbi_hantei");
    int y = gps.date.year();
    int m = gps.date.month();
    int d = gps.date.day();
    int h = gps.time.hour();
    int mi = gps.time.minute();
    int s = gps.time.second();
    int jst =(gps.time.hour() + 9) % 24;
    
    Serial.print(y);Serial.print('/');
    Serial.print(m);Serial.print('/');
    Serial.print(d);Serial.print(' ');
    Serial.print(h);Serial.print(':');
    Serial.print(mi);Serial.print(':');
    Serial.print(s);Serial.print(' ');
    Serial.print(jst);Serial.println(' ');
    
      int days = youbi(y,m,d);//GPSで取得した日付けを入れる
      int flag = 0;

     Serial.print("Youbi is "); Serial.println(days);
 

      int utc_min,utc_on_min;
      utc_min = h * 60 + mi; //現在時刻を　午前０時からの通算分で表す。
      utc_on_min = LIGHTUP_HOUR * 60 +LIGHTUP_MIN; //light up 時刻を　午前０時からの通算分で表す。
      
       Serial.print(utc_min);Serial.print("  ");
       Serial.print(utc_on_min);Serial.println(' ');
      
      if(is_suiyo(y,m,d,ONDAY) != 1){
       sleep_wait(5); //slppeを利用した　wait関数
      }
      else
      if(utc_min >= utc_on_min ){ // ライトアップの条件に一致した条件に一致した
        Serial.println("light up");
        digitalWrite(RL_SET,HIGH);delay(1000);digitalWrite(RL_SET,LOW); //点灯
        
        Serial.println("wait");
       // sleep_wait(60*ON_HOUR); //slppeを利用した　wait関数
        
       delay(1000l*60l*60l*ON_HOUR);    //強制的に5時間点灯　　
   
        digitalWrite(RL_RESET,HIGH);delay(500);digitalWrite(RL_RESET,LOW); //消灯
        Serial.println("light down");
  //       sleep_wait(60*ON_HOUR); //slppeを利用した　wait関数
      delay(1000l*60l*60l*5l);    //5時間消灯　消した後にすぐにつくので入れた。
          Serial.println("light down");
    //     sleep_wait(60*ON_HOUR); //slppeを利用した　wait関数
       delay(1000l*60l*60l*5L);    //5時間消灯　消した後にすぐにつくので入れた。
          Serial.println("light down");
   //       sleep_wait(60*1L); //slppeを利用した　wait関数
      delay(1000l*60l*60l*1L);    //1時間消灯　消した後にすぐにつくので入れた。
      }
      else  return;
    
  }
}


int youbi(int y,int m,int d)
{
  if(m==1 || m==2){
    y--;
    m+=12;
  }
  return ((y + y/4 - y/100 + y/400 + (13 * m + 8)/5 + d)%7);
}


//曜日判定　0:日曜～6:土曜
int is_suiyo(int _year, int _month, int _day, int ONday)
{
  int _youbi = youbi(_year,_month,_day);
 
  if(_youbi == ONday){
      Serial.print("Today is ON day!  ");
    return 1;
  }
  else 
    return 0;

}

void setup()
{
  
  // initialize the pushbutton pin as an input:
  pinMode(buttonPin, INPUT);  //use 4
  
   pinMode(RL_SET, OUTPUT);  // use 12
   pinMode(RL_RESET, OUTPUT); //use 13
 
  digitalWrite(RL_SET,LOW);digitalWrite(RL_RESET,LOW);delay(500);
  digitalWrite(RL_RESET,HIGH);delay(500);digitalWrite(RL_RESET,LOW);
  
  Serial.begin(115200);
  ss.begin(GPSBaud); //pin use 2,3 

  Serial.println(F("FullExample.ino"));
  Serial.println(F("An extensive example of many interesting TinyGPS++ features"));
  Serial.print(F("Testing TinyGPS++ library v. ")); Serial.println(TinyGPSPlus::libraryVersion());
  Serial.println(F("by Mikal Hart"));
  Serial.println();
  Serial.println(F("Sats HDOP Latitude   Longitude   Fix  Date       Time     Date Alt    Course Speed Card  Distance Course Card  Chars Sentences Checksum"));
  Serial.println(F("          (deg)      (deg)       Age                      Age  (m)    --- from GPS ----  ---- to London  ----  RX    RX        Fail"));
  Serial.println(F("---------------------------------------------------------------------------------------------------------------------------------------"));
 ss.end();
 
 slcd.begin(); //pin use 6,7
  // Print a message to the LCD.
  slcd.print("hello, world!");
  //slcd.end();
  
  ss.begin(GPSBaud);

}

void loop()
{
  static const double LONDON_LAT = 51.508131, LONDON_LON = -0.128002;
  int buttonState = 0;         // variable for reading the pushbutton status

   buttonState = digitalRead(buttonPin);
   
  // check if the pushbutton is pressed.
  // if it is, the buttonState is HIGH:
  if (buttonState == HIGH) { //　ボタンがおされたら、強制的に明りをつける 2015-02-24 H.Iire 
        Serial.println("light up");
        digitalWrite(RL_SET,HIGH);delay(1000);digitalWrite(RL_SET,LOW); //点灯
        
        Serial.println("wait");
       // sleep_wait(1);//10分点灯
       delay(1000l*60l*30 );    //30分点灯
   
        digitalWrite(RL_RESET,HIGH);delay(500);digitalWrite(RL_RESET,LOW); //消灯
        Serial.println("light down");
  }
  
  printInt(gps.satellites.value(), gps.satellites.isValid(), 5);
  printInt(gps.hdop.value(), gps.hdop.isValid(), 5);
  printFloat(gps.location.lat(), gps.location.isValid(), 11, 6);
  printFloat(gps.location.lng(), gps.location.isValid(), 12, 6);
  printInt(gps.location.age(), gps.location.isValid(), 5);
  printDateTime(gps.date, gps.time);
  printFloat(gps.altitude.meters(), gps.altitude.isValid(), 7, 2);
  printFloat(gps.course.deg(), gps.course.isValid(), 7, 2);
  printFloat(gps.speed.kmph(), gps.speed.isValid(), 6, 2);
  printStr(gps.course.isValid() ? TinyGPSPlus::cardinal(gps.course.value()) : "*** ", 6);

  unsigned long distanceKmToLondon =
    (unsigned long)TinyGPSPlus::distanceBetween(
      gps.location.lat(),
      gps.location.lng(),
      LONDON_LAT, 
      LONDON_LON) / 1000;
  printInt(distanceKmToLondon, gps.location.isValid(), 9);

  double courseToLondon =
    TinyGPSPlus::courseTo(
      gps.location.lat(),
      gps.location.lng(),
      LONDON_LAT, 
      LONDON_LON);

  printFloat(courseToLondon, gps.location.isValid(), 7, 2);

  const char *cardinalToLondon = TinyGPSPlus::cardinal(courseToLondon);

  printStr(gps.location.isValid() ? cardinalToLondon : "*** ", 6);

  printInt(gps.charsProcessed(), true, 6);
  printInt(gps.sentencesWithFix(), true, 10);
  printInt(gps.failedChecksum(), true, 9);
  Serial.println();
 
  //printValue(); 
     slcdPrint(); //シリアルLCD に文字を出力
 
 //Light up ? 

  smartDelay(1000); //1秒待ち　GPS受信機からはこの時間で実施。
  SW_loop();
 
  if (millis() > 5000 && gps.charsProcessed() < 10)
    Serial.println(F("No GPS data received: check wiring"));
}//end of loop()



/* 2015-2-28 追加　H.Irie
 パワーダウンモードでdelayする関数のデモ。
 パワーダウン中のCPUの消費電流は約28μA（Atmega328P＠16MHz,5V)
 ADCの電源がONにならないバグを修正（すーさんありがとうございます）
 2014/11/17 ラジオペンチ
 http://radiopench.blog96.fc2.com/
 */
 
 // ここから下を全て使う

void delayWDT(unsigned long t) {        // パワーダウンモードでdelayを実行
  delayWDT_setup(t);                    // ウォッチドッグタイマー割り込み条件設定
  ADCSRA &= ~(1 << ADEN);               // ADENビットをクリアしてADCを停止（120μA節約）
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);  // パワーダウンモード
  sleep_enable();

  sleep_mode();                         // ここでスリープに入る

  sleep_disable();                      // WDTがタイムアップでここから動作再開 
  ADCSRA |= (1 << ADEN);                // ADCの電源をON (|=が!=になっていたバグを修正2014/11/17)

}

void delayWDT_setup(unsigned int ii) {  // ウォッチドッグタイマーをセット。
  // 引数はWDTCSRにセットするWDP0-WDP3の値。設定値と動作時間は概略下記
  // 0=16ms, 1=32ms, 2=64ms, 3=128ms, 4=250ms, 5=500ms
  // 6=1sec, 7=2sec, 8=4sec, 9=8sec
  byte bb;
  if (ii > 9 ){                         // 変な値を排除 8秒以上の待ち時間はなし
    ii = 9;
  }
  bb =ii & 7;                           // 下位3ビットをbbに
  if (ii > 7){                          // 7以上（7.8,9）なら
    bb |= (1 << 5);                     // bbの5ビット目(WDP3)を1にする
  }
  bb |= ( 1 << WDCE );

  MCUSR &= ~(1 << WDRF);                // MCU Status Reg. Watchdog Reset Flag ->0
  // start timed sequence
  WDTCSR |= (1 << WDCE) | (1<<WDE);     // ウォッチドッグ変更許可（WDCEは4サイクルで自動リセット）
  // set new watchdog timeout value
  WDTCSR = bb;                          // 制御レジスタを設定
  WDTCSR |= _BV(WDIE);
} 

ISR(WDT_vect) {                         // WDTがタイムアップした時に実行される処理
  //  wdt_cycle++;                        // 必要ならコメントアウトを外す
}



void sleep_wait(unsigned int imin) // imin ※　60sec 
{
  unsigned int i,j;
  j=imin*15;
  
  for(i=0;i<j;i++)
  {
      delayWDT_setup(8); // wait in sleep using wdt　4sec
    
  }
  
  
}





