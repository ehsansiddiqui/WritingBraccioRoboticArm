
#include <Braccio.h>
#include <Servo.h>
#include <math.h>
#include <stdfix.h>


Servo base;
Servo shoulder;
Servo elbow;
Servo wrist_rot;
Servo wrist_ver;
Servo gripper;

//the unit of xy is cm
volatile double x_axis = 0;
volatile double y_axis = 14;
volatile double a0_value = 0;
volatile double a1_value = 0;
volatile double a2_value = 0;
unsigned int system_time = 0;
unsigned int increase_flag = 0;
volatile double radius;
double font_size = 1;
int state = 0;
//modes

volatile double tan_value[250] = {-1.0000,-0.9930,-0.9861,-0.9793,-0.9725,-0.9657,-0.9590,-0.9523,-0.9457,-0.9391,-0.9325,-0.9260,-0.9195,-0.9131,-0.9067,-0.9004,-0.8941,-0.8878,-0.8816,-0.8754,-0.8693,-0.8632,-0.8571,-0.8511,-0.8451,-0.8391,-0.8332,-0.8273,-0.8214,-0.8156,-0.8098,-0.8040,-0.7983,-0.7926,-0.7869,-0.7813,-0.7757,-0.7701,-0.7646,-0.7590,-0.7536,-0.7481,-0.7427,-0.7373,-0.7319,-0.7265,-0.7212,-0.7159,-0.7107,-0.7054,-0.7002,-0.6950,-0.6899,-0.6847,-0.6796,-0.6745,-0.6694,-0.6644,-0.6594,-0.6544,-0.6494,-0.6445,-0.6395,-0.6346,-0.6297,-0.6249,-0.6200,-0.6152,-0.6104,-0.6056,-0.6009,-0.5961,-0.5914,-0.5867,-0.5820,-0.5774,-0.5727,-0.5681,-0.5635,-0.5589,-0.5543,-0.5498,-0.5452,-0.5407,-0.5362,-0.5317,-0.5272,-0.5228,-0.5184,-0.5139,-0.5095,-0.5051,-0.5008,-0.4964,-0.4921,-0.4877,-0.4834,-0.4791,-0.4748,-0.4706,-0.4663,-0.4621,-0.4578,-0.4536,-0.4494,-0.4452,-0.4411,-0.4369,-0.4327,-0.4286,-0.4245,-0.4204,-0.4163,-0.4122,-0.4081,-0.4040,-0.4000,-0.3959,-0.3919,-0.3879,-0.3839,-0.3799,-0.3759,-0.3719,-0.3679,-0.3640,-0.3600,-0.3561,-0.3522,-0.3482,-0.3443,-0.3404,-0.3365,-0.3327,-0.3288,-0.3249,-0.3211,-0.3172,-0.3134,-0.3096,-0.3057,-0.3019,-0.2981,-0.2943,-0.2905,-0.2867,-0.2830,-0.2792,-0.2754,-0.2717,-0.2679,-0.2642,-0.2605,-0.2568,-0.2530,-0.2493,-0.2456,-0.2419,-0.2382,-0.2345,-0.2309,-0.2272,-0.2235,-0.2199,-0.2162,-0.2126,-0.2089,-0.2053,-0.2016,-0.1980,-0.1944,-0.1908,-0.1871,-0.1835,-0.1799,-0.1763,-0.1727,-0.1691,-0.1655,-0.1620,-0.1584,-0.1548,-0.1512,-0.1477,-0.1441,-0.1405,-0.1370,-0.1334,-0.1299,-0.1263,-0.1228,-0.1192,-0.1157,-0.1122,-0.1086,-0.1051,-0.1016,-0.0981,-0.0945,-0.0910,-0.0875,-0.0840,-0.0805,-0.0769,-0.0734,-0.0699,-0.0664,-0.0629,-0.0594,-0.0559,-0.0524,-0.0489,-0.0454,-0.0419,-0.0384,-0.0349,-0.0314,-0.0279,-0.0244,-0.0209,-0.0175,-0.0140,-0.0105,-0.0070,-0.0035,0.0000,0.0035,0.0070,0.0105,0.0140,0.0175,0.0209,0.0244,0.0279,0.0314,0.0349,0.0384,0.0419,0.0454,0.0489,0.0524,0.0559,0.0594,0.0629,0.0664,0.0699,0.0734,0.0769,0.0805,0.0840}; 

volatile double length[50] = {20.2,20.1,20.0,19.9,19.8,19.7,19.6,19.5,19.4,19.3,19.2,19.1,19.0,18.9,18.8,18.7,18.6,18.5,18.4,18.3,18.2,18.1,18.0,17.9,17.8,17.7,17.6,17.5,17.4,17.3,17.2,17.1,17.0,16.9,16.8,16.7,16.6,16.5,16.4,16.3,16.2,16.1,16.0,15.9,15.8,15.7,15.6,15.5,15.4,15.3}; 

//volatile int angle1[50] = {3756,3618,3551,3503,3462,3427,3395,3365,3338,3314,3290,3266,3245,3225,3206,3187,3167,3150,3132,3117,3101,3085,3069,3054,3040,3026,3011,2999,2986,2972,2959,2948,2935,2922,2911,2900,2889,2878,2867,2856,2844,2835,2824,2814,2803,2793,2784,2774,2765,2755};

//volatile int angle0[601] = {1832,1838,1843,1849,1855,1860,1866,1871,1877,1882,1888,1893,1899,1905,1910,1916,1921,1927,1932,1938,1943,1949,1955,1960,1966,1971,1977,1982,1988,1993,1999,2005,2010,2016,2021,2027,2032,2038,2043,2049,2055,2060,2066,2071,2077,2082,2088,2093,2099,2105,2110,2116,2121,2127,2132,2138,2143,2149,2155,2160,2166,2171,2177,2182,2188,2193,2199,2205,2210,2216,2221,2227,2232,2238,2243,2249,2255,2260,2266,2271,2277,2282,2288,2293,2299,2305,2310,2316,2321,2327,2332,2338,2343,2349,2355,2360,2366,2371,2377,2382,2388,2393,2399,2405,2410,2416,2421,2427,2432,2438,2443,2449,2455,2460,2466,2471,2477,2482,2488,2493,2499,2505,2510,2516,2521,2527,2532,2538,2543,2549,2555,2560,2566,2571,2577,2582,2588,2593,2599,2605,2610,2616,2621,2627,2632,2638,2643,2649,2655,2660,2666,2671,2677,2682,2688,2693,2699,2705,2710,2716,2721,2727,2732,2738,2743,2749,2755,2760,2766,2771,2777,2782,2788,2793,2799,2805,2810,2816,2821,2827,2832,2838,2843,2849,2855,2860,2866,2871,2877,2882,2888,2893,2899,2905,2910,2916,2921,2927,2932,2938,2943,2949,2955,2960,2966,2971,2977,2982,2988,2993,2999,3005,3010,3016,3021,3027,3032,3038,3043,3049,3055,3060,3066,3071,3077,3082,3088,3093,3099,3105,3110,3116,3121,3127,3132,3138,3143,3149,3155,3160,3166,3171,3177,3182,3188,3193,3199,3205,3210,3216};

unsigned int sita12;                                                            //select the angle12 table
unsigned int sita0;                                                             //select the angle0 table

int getsita0(double x, double y){
    int k;
    double tan_sita;
    tan_sita = -x/y;
    for(k = 0; k <= 311; k++){
        if(tan_sita >= tan_value[k] && tan_sita <= tan_value[k+1])
            break;
    }
    if(k >= 311)
        k = 311;
    return k;
}

int getsita12(double x, double y){
    int i;
    double radius;
    radius = sqrt(x*x+y*y);
    for(i = 0; i <= 115; i++){
        if(radius <= length[i] && radius >= length[i+1])
            break;
    }
    if(i >= 115)
        i = 115;
    return i;
}

void setxy(double x, double y, int down){
    int a,b;
    int i;
    a = getsita12(x,y);
    b = getsita0(x,y);
//    SetDCOC2PWM(angle1[a]-500);                                                //servo1
//    delay(50);
//    SetDCOC3PWM(angle0[b]);                                                 //servo0
//    SetDCOC1PWM(angle2[a]);                                                //servo2
//    SetDCOC4PWM(angle3[a]+300);                                            //servo3

      Serial.println("Setxy: a");
     Serial.println(a);
     Serial.println("Setxy: b");
     Serial.println(b);  

//    Braccio.ServoMovement(50,angle2[a],angle1[a]-500,angle0[b], angle3[a]+300, 90, 73);
//    delay(50);
//    if(down == 1){                                                              //put down the pen
//        for(i = 0;i <= 24; i++){
////            SetDCOC2PWM(angle1[a]-500+20*i);    //servo1
//              Braccio.ServoMovement(50,angle2[a],angle1[a]-500+20*i,angle0[b], angle3[a]+300, 90,  73);
//            delay(50);
//        }
//    }
    delay(1000);
}

void writexy(double x, double y){
//  Serial.println("Running");
    int a,b;
    a = getsita12(x,y);
    b = getsita0(x,y);
//    SetDCOC3PWM(angle0[b]);                                                 //servo0
//    SetDCOC2PWM(angle1[a]);                                                //servo1
//    SetDCOC1PWM(angle2[a]);                                                //servo2
//    SetDCOC4PWM(angle3[a]+300);                                            //servo3
Serial.println("nritexy: a"); 
      Serial.println(a);
        Serial.println("mritexy: b");
      Serial.println(b);  
//      Braccio.ServoMovement(20, angle0[b],angle1[a],angle2[a], angle3[a]+400, 0,  73);
    delay(100);
}

void drawline(double x1, double y1, double x2, double y2){
    double stepx,stepy;
    int i;
    stepx = (x2-x1)/10.0;
    stepy = (y2-y1)/10.0;
    writexy(x1,y1);
    for(i=0;i<=10;i++){
        x1 = x1 + stepx;
        y1 = y1 + stepy;
        writexy(x1,y1);
    }
    
}

void draw_symbol(char a, double x, double y){
    switch(a){
            
            //alphabets
        case 'A':
            setxy(x,y-font_size*2,1);
            drawline(x,y-font_size*2,x+font_size/2,y);
            drawline(x+font_size/2,y,x+font_size,y-font_size*2);
            setxy(x,y-font_size,1);
            drawline(x,y-font_size,x+font_size,y-font_size);
            setxy(x+font_size,y-font_size,0);
            break;
            
        case 'B':
            setxy(x,y,1);
            drawline(x,y,x,y - font_size*2);
            drawline(x,y - font_size*2,x+font_size,y-3 * font_size/2);
            drawline(x+font_size,y-3 * font_size/2,x,y-font_size);
            drawline(x,y-font_size,x + font_size,y-font_size/2);
            drawline(x + font_size,y-font_size/2,x,y);
            setxy(x,y,0);
            break;
            
        case 'C':
            setxy(x+font_size,y,1);
            drawline(x+font_size,y,x,y);
            drawline(x,y,x,y-font_size*2);
            drawline(x,y-font_size*2,x+font_size,y-font_size*2);
            setxy(+font_size,y-font_size*2,0);
            break;
            
        case 'D':
            setxy(x,y,1);
            drawline(x,y,x,y - font_size*2);
            drawline(x,y - font_size*2,x+font_size,y-3 * font_size/2);
            drawline(x+font_size,y-3 * font_size/2,x + font_size,y-font_size/2);
            drawline(x + font_size,y-font_size/2,x ,y);
            setxy(x,y,0);
            break;
            
        case 'E':
            setxy(x+font_size,y,1);
            drawline(x+font_size,y,x,y);
            drawline(x,y,x,y-font_size*2);
            drawline(x,y-font_size*2,x+font_size,y-font_size*2);
            setxy(x+font_size,y-font_size,1);
            drawline(x+font_size,y-font_size,x,y-font_size);    
            setxy(x,y-font_size,0);
            break;
        
        case 'F':
            setxy(x+font_size,y,1);
            drawline(x+font_size,y,x,y);
            drawline(x,y,x,y-font_size*2);
            setxy(x+font_size,y-font_size,1);
            drawline(x+font_size,y-font_size,x,y-font_size);    
            setxy(x,y-font_size,0);
            break;
            
        case 'G':
            setxy(x+font_size,y,1);
            drawline(x+font_size,y,x,y);
            drawline(x,y,x,y-font_size*2);
            drawline(x,y-font_size*2,x+font_size,y-font_size*2);
            drawline(x+font_size,y-font_size*2,x+font_size,y-font_size);
            drawline(x+font_size,y-font_size,x +font_size/2,y-font_size);
            setxy(x+font_size/2,y-font_size,0);
            break;
            
        case 'H':
            setxy(x,y,1);
            drawline(x,y,x,y-font_size*2);
            setxy(x+font_size,y,1);
            drawline(x + font_size,y,x+font_size,y-font_size*2);
            setxy(x+font_size,y-font_size,1);
            drawline(x+font_size,y-font_size,x,y-font_size);
            setxy(x,y-font_size,0);
            break;
            
        case 'I':
            setxy(x,y,1);
            drawline(x,y,x+font_size,y);
            setxy(x+font_size/2,y,1);
            drawline(x+font_size/2,y,x+font_size/2,y-font_size*2);
            setxy(x+font_size,y-font_size*2,1);
            drawline(x+font_size,y-font_size*2,x,y-font_size*2);    
            setxy(x,y-font_size*2,0);
            break;
            
        case 'J':
            setxy(x+font_size,y,1);
            drawline(x+font_size,y,x+font_size,y-font_size*2);
            drawline(x+font_size,y-font_size*2,x+0.2*font_size,y-font_size*2);
            drawline(x,y-font_size*2,x,y-3*font_size/2);    
            setxy(x,y-font_size,0);
            break;
            
        case 'K':
            setxy(x,y,1);
            drawline(x,y,x,y - font_size*2);
            setxy(x+font_size,y-font_size*2,0);
            drawline(x+font_size,y-font_size*2,x,y-font_size);
            drawline(x,y-font_size,x+font_size,y);
            setxy(x+font_size,y,0);
            break;
            
        case 'L':
            setxy(x,y,1);
            drawline(x,y,x,y-font_size*2);
            drawline(x,y-font_size*2,x+font_size,y-font_size*2);
            setxy(x+font_size,y-font_size*2,0);
            break;
            
        case 'M':
            setxy(x,y-font_size*2,1);
            drawline(x,y-font_size*2,x+0.25*font_size,y);
            drawline(x+0.25*font_size,y,x+0.5*font_size,y-2*font_size);
            drawline(x+0.5*font_size,y-2*font_size,x+0.75*font_size,y);
            drawline(x+0.75*font_size,y,x+font_size,y-2*font_size);
            setxy(x+font_size,y-2*font_size,0);
            break;
            
        case 'N':
            setxy(x,y-font_size*2,1);
            drawline(x,y-font_size*2,x,y);
            drawline(x,y,x+font_size,y-font_size*2);
            drawline(x+font_size,y-font_size*2,x+font_size, y);
            setxy(x+font_size, y,0);
            break;
            
        case 'O':
            setxy(x,y,1);
            drawline(x,y,x+font_size,y);
            drawline(x+font_size,y,x+font_size,y-font_size*2);
            drawline(x+font_size,y-font_size*2,x,y-font_size*2);
            drawline(x,y-font_size*2,x,y);   
            setxy(x,y,0);
            break;
            
        case 'P':
            setxy(x,y-font_size,1);
            drawline(x,y-font_size,x+font_size,y-font_size);
            drawline(x+font_size,y-font_size,x+font_size,y);
            drawline(x+font_size,y,x,y);
            drawline(x,y,x,y-font_size*2);
            setxy(x,y-font_size*2,0);
            break;
            
        case 'Q':
            setxy(x+font_size,y-font_size,1);
            drawline(x+font_size,y-font_size,x,y-font_size);
            drawline(x,y-font_size,x,y);
            drawline(x,y,x+font_size,y);
            drawline(x+font_size,y,x+font_size,y-font_size*2);
            setxy(x+font_size,y-font_size*2,0);
            break;
            
        case 'R':
            setxy(x,y-font_size*2,1);
            drawline(x,y-font_size*2,x,y);
            drawline(x,y,x+font_size,y);
            drawline(x+font_size,y,x+font_size,y-font_size);
            drawline(x+font_size,y-font_size,x,y-font_size);
            drawline(x,y-font_size,x+font_size,y-font_size*2);
            setxy(x+font_size,y-font_size*2,0);
            break;
            
        case 'S':
            setxy(x+font_size,y,1);
            drawline(x+font_size,y,x,y);
            drawline(x,y,x,y-font_size);
            drawline(x,y-font_size,x+font_size,y-font_size);
            drawline(x+font_size,y-font_size,x+font_size,y-font_size*2);
            drawline(x+font_size,y-font_size*2,x,y-font_size*2);
            setxy(x,y-font_size*2,0);
            break;
            
        case 'T':
            setxy(x,y,1);
            drawline(x,y,x+font_size,y);
            setxy(x+font_size/2,y,1);
            drawline(x+font_size/2,y,x+font_size/2,y-2*font_size);
            setxy(x+font_size/2,y-2*font_size,0);
            break;
            
        case 'U':
            setxy(x,y,1);
            drawline(x,y,x,y-2*font_size);
            drawline(x,y-2*font_size,x+font_size,y-2*font_size);
            drawline(x+font_size,y-2*font_size,x+font_size,y);
            setxy(x+font_size,y,0);
            break;
            
        case 'V':
            setxy(x,y,1);
            drawline(x,y,x+1/2*font_size,y-2*font_size);
            drawline(x+1/2*font_size,y-2*font_size,x+font_size,y);
            setxy(x+font_size,y,0);
            break;
            
        case 'W':
            setxy(x,y,1);
            drawline(x,y,x+0.25*font_size,y-2*font_size);
            drawline(x+0.25*font_size,y-2*font_size,x+0.5*font_size,y);
            drawline(x+0.5*font_size,y,x+0.75*font_size,y-2*font_size);
            drawline(x+0.75*font_size,y-2*font_size,x+font_size,y);
            setxy(x+font_size,y,0);
            break;
            
        case 'X':
            setxy(x,y,1);
            drawline(x,y,x+font_size,y-font_size*2);
            setxy(x+font_size,y,1);
            drawline(x+font_size,y,x,y-font_size*2);
            setxy(x,y-font_size*2,0);
            break;
            
        case 'Y':
            setxy(x,y,1);
            drawline(x,y,x+font_size/2,y-font_size);
            drawline(x+font_size/2,y-font_size,x+font_size,y);
            drawline(x+font_size,y,x+font_size/2,y-font_size);
            drawline(x+font_size/2,y-font_size,x+font_size/2,y-font_size*2);
            setxy(x+font_size/2,y-font_size*2,0);
            break;
            
        case 'Z':
            setxy(x,y,1);
            drawline(x,y,x+font_size,y);
            drawline(x+font_size,y,x+font_size/2,y-font_size);
            drawline(x+font_size/2,y-font_size,x,y-font_size*2);
            drawline(x,y-font_size*2,x+font_size,y-font_size*2);
            setxy(x+font_size,y-font_size*2,0);
            break;
    }
}


void setup() {
  // put your setup code here, to run once:
  Braccio.begin();
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
   draw_symbol('C',0.2,16);
   exit(0);

}
