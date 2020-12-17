class HC_SR04 
{
  private:

          int TrigPin;
          int EchoPin;
          long Duration;
          int Distance;

  public:

          int ObsDistance;
          
  void HCSetup(int trigPin, int echoPin)
    {
      long duration;
      int distance;
      TrigPin = trigPin;
      EchoPin = echoPin;
      Duration = duration;
      Distance = distance;

      pinMode(TrigPin, OUTPUT);
      pinMode(EchoPin, INPUT);
    }

  void HCLoop() 
    {
      digitalWrite(TrigPin,LOW);  
      delayMicroseconds(2);
      digitalWrite(TrigPin,HIGH);
      delayMicroseconds(10);  
      digitalWrite(TrigPin,LOW);
      Duration = pulseIn(EchoPin,HIGH);
      Distance = Duration*0.034/2;
      ObsDistance = Distance;
      Serial.println(ObsDistance);
    }

};

HC_SR04 HC;
