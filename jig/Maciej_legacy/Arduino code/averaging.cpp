int TempMemory[] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
int TempMemoryIndex;
unsigned long ReadingCounter = 0;
int Sum = 0;
int Average;


int AverageReading(int val){
      ReadingCounter++;
      TempMemoryIndex = ReadingCounter%20 - 1;
      if(TempMemoryIndex == -1)
      TempMemoryIndex = 19;
      Sum = Sum - TempMemory[TempMemoryIndex];
      TempMemory[TempMemoryIndex] = val;
      Sum = Sum + TempMemory[TempMemoryIndex];
      Average = Sum/20;
      return Average;
}

int valAverage = 0;

void loop() {
 
  val = analogRead(InputPin);            // reads the value of the potentiometer (value between 0 and 1023)
  valAverage = AverageReading(val); 
  //updateServos
  delay(15);                           // waits for the servo to get there
}