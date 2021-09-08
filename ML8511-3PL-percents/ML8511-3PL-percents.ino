#define LED 13
#define EXTREMES 10

int minValues[EXTREMES];
int maxValues[EXTREMES];

void setup()
{
	pinMode(LED, OUTPUT);
	digitalWrite(LED, HIGH);
	int value = 0;
	for(int i = 0;i<EXTREMES;i++)
	{
		maxValues[i] = 0;
		minValues[i] = 1024;
	}
	for(int i=0;i<800;i++)
	{
		
	}
	digitalWrite(LED, HIGH);
}

void loop()
{
}

void firstTen()
{
	
	for(i=0;i<20;i++)
	{
		
	}
}

int checkMinExtreme(int n)
{
	if(n>minValues(0)){return n;}
	else
	{
		int j = 0;
		while(j<=EXTREMES && n>minValues(j)){j++;}
		int rV = minValues(0);
		for(int k=0;k<j-1;k++){minValues(k) = minValues(k+1);}
		minValues(j-1) = n;
		return rV;
	}
}

int checkMaxExtreme(int n)
{
	if(n<maxValues(0)){return n;}
	else
	{
		int j = 0;
		while(j<=EXTREMES && n>maxValues(j)){j++;}
		int rV = maxValues(0);
		for(int k=0;k<j-1;k++){maxValues(k) = maxValues(k+1);}
		maxValues(j-1) = n;
		return rV;
	}
}
