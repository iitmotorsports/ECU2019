int i =2;
int pg = 1;
int bl = 100;
int lmt = 1;
int rmt = 1;
int lmct = 1;
int rmct = 1;
int aap = 1;
int soc = 1;
int DCbc = 1;
int f = 1;
int ttt = 1;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(13,OUTPUT);
}
void loop() {
  digitalWrite(13,HIGH);
  i=(i+1)%100;
  pg= (pg+1)%99;
  bl= (bl-1)%100;
  lmt=(lmt+1)%100;
  rmt=(rmt+1)%100;
  lmct=(lmct+1)%100;
  rmct=(rmct+1)%100;
  aap=(aap+1)%100;
  DCbc=(DCbc+1)%100;
  String s = "S " + (String)i + " " + (String)pg + " " + (String)bl + " "
  + (String)lmt + " "+ (String)rmt + " "+ (String)lmct + " "+ (String)rmct + " "
  + (String)aap + " " + (String)DCbc + " ";
  Serial.println(s);
  delay(1000);
}
