#define READPIN A0;
#define REFPIN A1;

void setup()
{
	Serial.begin(9600);
	Serial.println("Vitejte!");
	Serial.println("Za chvili se vam nize zacnou objevovat hodnoty UV:");
}

void loop()
{
}

float voltageFromAnRead(int AnRead, int refVoltage)
{
	float x = 3.3 * AnRead/refVoltage;
	return x;
}

float UVFromVoltage(float Voltage)
{
	if (Voltage<0.96){return 0.0;}
	else
	{
		float UV=(Voltage - 0.96)/1.84*15.0;
		return UV;
	}
}

int averageAnalogRead(int pin)
{
	int minValue = analogRead(pin);
	int maxValue = minValue;
	
}
