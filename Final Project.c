#define MINDIST 30 //Minimum distance to look for objects, filters out the rest
#define TURNSPEED 4 //speed at which robot turns. Lower means slower scan but possibly more accurate
#define MAXOBJECTS 10 //set length of object array. Can be greater than the number of objects found, but not less than
#define W 178 //screen width
#define H 128 //screen height
#define C 6.1 //distance from the axis of rotation to the gyro sensor. Major source of error @ distance and large angles

//This structure includes all the information required for each block.
//It is made in the think function using the results of the scan
typedef struct{
	int angle;
	float distance;
	float start[2];
	float end[2];
	bool enabled;
}object;

//Main array of blocks
object blocks[MAXOBJECTS];

float dist=0; //float so calibration value C can be more precise
int gyro=0; //rotation from x-axis in degrees
float distScan[360]={0}; //store scan results

void scan(); //scan environment
void think(); //process data
void display(object *blockArray); //display map

task main()
{
	// setup sensors
	SensorType[S1] = sensorEV3_Gyro;
	SensorType[S2] = sensorEV3_Ultrasonic;
	
	scan();
	think();
	wait1Msec(30000);
}

void think()
{
	//prevState and currState used to find the edge of each block by finding the edges
	bool prevState=false;
	bool currState;
	//Counts number of blocks so you don't max out the block array
	int count=0;
	//make it look like its doing important stuff
	displayBigTextLine(6,"Initializing");
	wait1Msec(1000);
	//loop through every data point in scan array
	for(int i=0;i<360;i++)
	{
		//look for usable data points
		if(distScan[i]<MINDIST)
		{
			currState=true;
		}
		else
		{
			currState=false;
		}
		if(currState!=prevState && count<MAXOBJECTS)
		{
			//found the start of a block! Calculate values
			if(currState)
			{
				//displayBigTextLine(2,"Start:%d,%f",i,distScan[i]);
				//calculate distances
				float y=(distScan[i]*cosDegrees(i));
				float x=(distScan[i]*sinDegrees(-i));
				//set info
				blocks[count].start[0]=x;
				blocks[count].start[1]=y;
				blocks[count].enabled=true;
				blocks[count].angle=i;
				//wait1Msec(1000);
			}
			//The end of the block :( Setup more stuff
			else
			{
				//displayBigTextLine(2,"End:%d,%f",i-1,distScan[i-1]);
				//calculate distnaces
				float y=(distScan[i-1]*cosDegrees(i-1));
				float x=(distScan[i-1]*sinDegrees(-i+1));
				
				//This is a filter. The sensor or code can glitch,
				//causing small blocks where there are, in fact, no blocks.
				int min=2; //min block length & width
				if ((abs(blocks[count].end[0]-x))<min || (abs(blocks[count].end[1]-y))<min)
				{
					blocks[count].start[0]=0;
					blocks[count].start[1]=0;
					blocks[count].end[0]=0;
					blocks[count].end[1]=0;
					blocks[count].angle=0;
					blocks[count].distance=0;
				}
				else{
					blocks[count].end[0]=x;
					blocks[count].end[1]=y;
					blocks[count].angle=(blocks[count].angle+(i-1))/2; //find middle angle
					blocks[count].distance=distScan[blocks[count].angle]; //find middle distance
					//wait1Msec(500);
					count++;
				}
			}
		}
		prevState=currState;
	}
	display(blocks); //loop is done! Display map
}

//DISPLAY FUNCTION
//Displays scan data in a map where 1.5px=1cm
void display(object *blockArray)
{
	eraseDisplay(); //clear screen
	displayTextLine(1,"    ENEMY TARGETS");
	for(int i=0;i<MAXOBJECTS;i++)
	{
		if(blockArray[i].enabled) //only display blocks that are actually there
		{
			//find line coordinates
			int x_i=W/2+1.5*(blockArray[i].start[0]);
			int y_i=H/2+1.5*(blockArray[i].start[1]);
			int x_f=W/2+1.5*(blockArray[i].end[0]);
			int y_f=H/2+1.5*(blockArray[i].end[1]);
			//draw line, make it 2px thick
			drawLine(x_i,y_i,x_f,y_f);
		}
	}
}

void scan()
{
	//old gyro is used to speed up program. The distance is saved only once per degree
	int oldGyro=-1;
	int initialGyro=SensorValue[S1];
	//loop until one complete rotation is done
	//make robot spin
	setMotorSpeed(motorB,TURNSPEED);
	setMotorSpeed(motorC,-TURNSPEED);
	while(abs(gyro)<360)
	{
		//find data from external sensors
		gyro=SensorValue[S1]-initialGyro;
		dist=SensorValue[S2]+C;
		//only record data for new angle (robot loops code faster than it rotates)
		if(abs(gyro)>abs(oldGyro))
		{
			int index=abs(gyro)-1; //make index positive and stay within array bounds
			if (dist<MINDIST)
			{
				//displayTextLine(1,":)"); for debugging
				setMotorSpeed(motorB,TURNSPEED-2);
				setMotorSpeed(motorC,-TURNSPEED+2);
				distScan[index]=dist; //save data
			}
			else
			{
				setMotorSpeed(motorB,TURNSPEED);
				setMotorSpeed(motorC,-TURNSPEED);
				//displayTextLine(1,":("); for debugging
				distScan[index]=MINDIST+1; //save point, but its useless data
			}
		}
		//display data on screen
		displayBigTextLine(2,"Gyro (deg.):");
		displayBigTextLine(5,"   %d",gyro);
		displayBigTextLine(9,"Dist (cm):");
		displayBigTextLine(12,"   %.1f",dist);
		oldGyro=gyro;
	}
	//stop motors
	setMotorSpeed(motorB,0);
	setMotorSpeed(motorC,0);
	//clear display
	eraseDisplay();
}
