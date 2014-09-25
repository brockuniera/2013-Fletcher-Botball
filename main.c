#include <stdio.h>
#include <stdlib.h>

//drivetrain ports
#define LFT 1
#define RGT 0
#define SPD 5

//claw: servo
#define GRAB 3
#define DROP 740
#define UP 1200
#define DOWN 1850

//camera stuff
//channels
#define GRN 0
#define ORNG 1
#define PIPEA 2
#define PIPEB 3

//basket
#define BASKET 0
#define IN 1788
#define OUT 597

//coordinates for ideal it centroid position
int GOALY = 81; 
int GOALX = 193; 
int XTOLER = 10; 
int YTOLER = 7;

point2 green;
point2 orange;
point2 myobj;

#define TOLER 7

//other stuff
#define ARGSIZE 25
#define getpos(x) get_motor_position_counter(x)
#define turn(x) move_to_position(TURN, 1000, x)
#define printval(x) printf("%d\n", x)

#define checkpos \
	while(!side_button()){\
		msleep(100);\
		update_green();\
		printpos();\
	}\
	while(side_button())

#include <pthread.h>

void testfuncs(int argc);

void search();
int update_obj(void);
int find_green(int xcood, int ycood, int delayms);
void grab_it();
void readinput(int argc, char **argv);
void print_objs(int chan);
void printpos(void);

int update_green();
//require update_green() to be done first
int get_ydist(void);
int get_xdeg(void);

#include "/kovan/botfuncs.h"
#include "/kovan/opencode.h"
#include "/kovan/line_follow.h"
/*
GOALY = atoi(argv[1]); //111
YTOLER = atoi(argv[2]); //3
GOALX = atoi(argv[3]); //200
XTOLER = atoi(argv[4]); //10
*/

//norman movement consts
#define WHEEL_DIAM 44.1 //diamater of wheel
#define WHEEL_RAD_DIST 75.0 //dist from center to interior of wheel, along its axis

void print_time(void){
	int i = 0;
	while(1){
		sleep(1);
		printf("Time is: %d\n", i++);
	}
}

void camera_update_forever(void){
	while(1){
		camera_update();
		msleep(70);
	}
}

void claw_wiggle(void){
	const int wiggle = 150;
	const int wigdelay = 200;

	set_servo_position(GRAB, DROP + 2 * wiggle);
	msleep(wigdelay);
	set_servo_position(GRAB, DROP + wiggle);
	msleep(wigdelay);
	set_servo_position(GRAB, DROP + 2 * wiggle);
	msleep(wigdelay);
	set_servo_position(GRAB, DROP + wiggle);
}

void basket_wiggle(void){
	const int wiggle = 150;
	const int wigdelay = 100;

	set_servo_position(BASKET, IN + 2 * wiggle);
	msleep(wigdelay);
	set_servo_position(BASKET, IN + wiggle);
	msleep(wigdelay);
	set_servo_position(BASKET, IN + 2 * wiggle);
	msleep(wigdelay);
	set_servo_position(BASKET, IN + wiggle);
}

void past_cube(void){
	/*
	line_follow_outside(2800);
	cbc_straight(1000, -60);
	cbc_spin(1000, 35);
	*/

	cbc_straight(1000, -35);
	cbc_spin(1000, 55);
	cbc_straight(1000, 80);

	cbc_arc(450, -215, -65); //speed, rad, deg
	cbc_straight(1000, 30);
}

int choose_equation(void){
	enum {A1, A2, B1, B2, C1, C2};
	enum {LDIV = 64, RDIV = 215};

	if(green.x == -1){
		printf("not a valid position\n");
		return -1;
	}

	if(green.x <= LDIV){
		return A1;
	}else if(green.x > LDIV && green.x < RDIV){ //b1 is really bad
		return B2;
	}else{
		return C1;
	}
}

//temporary equation tester
//we can find out what works where
int get_equation(int i){
	switch(i){
		case 0:
			printf("A1\n");
			return green.y * -.085 + .8708;
			break;
		case 1:
			printf("A2\n");
			return (green.x - 116) / 5.5;
			break;
		case 2:
			printf("B1\n");
			return (green.x - 149.25) / 30;
			break;
		case 3:
			printf("B2\n");
			//return (green.x - 147) / 18;
			return green.x * .0831 - 10.595;
			break;
		case 4:
			printf("C1 (this uses Ycood)\n");
			return (green.y + 456) / 50;
			break;
		case 5:
			printf("C2\n");
			return (green.x + 53) / 15;
			break;
		default:
			printf("%d is not a valid number\n", i);
			return 0;
			break;
	}
}

point2 pipe;
int update_pipe(){
	pipe.x = -1;
	pipe.y = -1;

	int area = 1000;
	int i;
	for(i = 0; i < get_object_count(PIPEA); i++){
		if(get_object_area(PIPEA, i) > area){
			area = get_object_area(PIPEA, i);
			pipe = get_object_centroid(PIPEA, i);
		}
	}
	return area;
}	

void pipe_follow(void){
	do{
		update_eyes();
		update_pipe();

		printf("PIPE X: %d PIPE Y: %d\n", pipe.x, pipe.y);

		if(pipe.x == -1){
			//arc left
			motor(LFT, 60);
			motor(RGT, 100);
		}else{
			//arc right
			motor(LFT, 100);
			motor(RGT, 10);
		}
	}while(eyes.l == W && eyes.r == W);
	stopmotors();
}

void go_to_line(int dir){
	dir = dir > 0 ? 1 : -1;

	motor(LFT, 83 * dir);
	motor(RGT, 100 * dir);
	do{
		update_eyes();
	}while(eyes.l == W && eyes.r == W);
	stopmotors();
}

void line_straight(void){
	go_to_line(1);

	motor(LFT, 40 * .7);
	motor(RGT, 40);
	do{
		update_eyes();

		if(eyes.l == W)
			freeze(LFT);
		if(eyes.r == W)
			freeze(RGT);
	}while(eyes.l == B || eyes.r == B);
}


void get_piles(int type){
	//pile 1
	line_follow_inside(-2);
	cbc_spin(1000, -20);
	search();

	//pile 2
	cbc_spin(1000, 65);
	cbc_straight(1000, 20);
	search();
	cbc_straight(1000, -20);

	//pile 3
	motor(RGT, 80);
	motor(LFT, -70);
	do{
		update_eyes();
	}while(eyes.r == W);
	do{
		update_eyes();
	}while(eyes.l == B);
	stopmotors();

	line_follow_outside(type);
	stopmotors();

	past_cube();

	search();
}

int main(int argc, char **argv){
	camera_open(MED_RES);
	//thread t = thread_create(&camera_update_forever);
	//thread_start(t);

	thread time = thread_create(&print_time);
	thread_start(time);

	//port, ticks/rev, eff, diam, radial distance
	build_left_wheel(RGT, 1060, 1.0 , WHEEL_DIAM, WHEEL_RAD_DIST);
	build_right_wheel(LFT, 1010, 0.930 , WHEEL_DIAM, WHEEL_RAD_DIST);

	enable_servos();
	set_servo_position(BASKET, IN);
	set_servo_position(GRAB, DROP);

	sleep(2);

	testfuncs(argc);


	//start
	go_to_line(-1);

	cbc_straight(1000, 30);
	cbc_spin(1000, -80);

	get_piles(-2);

	//end thing
	
	cbc_straight(1000, 190);
	cbc_spin(1000, -85);
	line_straight();

	//pile 4
	cbc_straight(1000, 20);
	search();

	cbc_straight(1000, 260);
	cbc_straight(1000, -45);

	cbc_spin(1000, 82);

	cbc_straight(1000, 150);

	motor(LFT, 88);
	motor(RGT, 100);
	msleep(2900);
	stopmotors();

	cbc_straight(1000, 200);
	cbc_spin(1000, 90);

	go_to_line(1);

	//halfway!

	//alt method
	/*
	cbc_spin(1000, 70);
	cbc_straight(1000, 100);
	cbc_straight(1000, -80);
	cbc_spin(1000, -75);

	pipe_follow();

	//fixes herding
	cbc_straight(1000, 160);
	go_to_line(-1);
	*/

	line_follow_outside(-4);

	//at orange pile of 4
	
	cbc_straight(1000, 70);
	cbc_spin(1000, 60);

	motor(LFT, 73);
	motor(RGT, 100);
	do{
		update_eyes();
	}while(eyes.l == W && eyes.r == W);
	stopmotors();

	get_piles(-3);

	//end thing
	cbc_straight(1000, 160);
	cbc_spin(1000, -85);
	line_straight();

	//last pile
	cbc_straight(1000, 80);
	search();

	cbc_straight(1000, 230);
	cbc_straight(1000, -55);

	cbc_spin(1000, 84);

	cbc_straight(1000, 150);

	motor(LFT, 79);
	motor(RGT, 100);
	msleep(2300);
	stopmotors();

	//pause for gold tribs
	printf("TRIBBLE TIME!\n");
	set_servo_position(GRAB, UP);
	sleep(5);

	//get to the line!
	cbc_straight(1000, 120);

	cbc_spin(1000, 70);

	go_to_line(1);

	line_follow_outside(5000);

	//handle the transport
	cbc_spin(1000, 135);

	cbc_straight(1000, -120);

	motor(RGT, -100);
	msleep(2500);

	cbc_straight(1000, -100);
	set_servo_position(BASKET, OUT);

}

void grab_it(){
	set_servo_position(GRAB, DOWN); msleep(300);
	mrp(LFT, 1000, 50);
	set_servo_position(GRAB, DOWN+100); msleep(200);
	block_motor_done(LFT);
	mrp(LFT, 1000, -50);
	block_motor_done(LFT);
	mrp(RGT, 1000, 50);
	block_motor_done(RGT);
	mrp(RGT, 1000, -50);
	block_motor_done(RGT);

	set_servo_position(GRAB, DOWN); msleep(300);

	set_servo_position(GRAB, DROP);
}

void search(void){
	//camera_update();
	const float prop = .2;
	int dist, deg, extra, extradeg;
	extra = extradeg = 0;

	update_green();
	while(get_object_count(GRN) == 0 || green.x == -1){
		update_green();
		if(get_object_count(ORNG) != 0){
			printf("centering on orange!\n");
			update_orange();
			if(orange.x > GOALX+XTOLER){
				motor(LFT, (orange.x - GOALX)/prop + 6);
			}else if(orange.x < GOALX-XTOLER){
				motor(RGT, (GOALX - orange.x)/prop + 6);
			}else if(orange.y < GOALY-YTOLER){
				int pwr = (GOALY - orange.y) / prop + 6;
				motor(LFT, pwr);
				motor(RGT, pwr);
			}else if(orange.y > GOALY+YTOLER){
				int pwr = (orange.y - GOALY ) / prop + 6;
				motor(LFT, -pwr);
				motor(RGT, -pwr);
			}
		}
		msleep(200);
	}
	stopmotors();

	msleep(150);
	update_green();
	printpos();

	if(green.y < 50){
		cbc_straight(1000, extra = 20);
		msleep(150);
		update_green();
		printpos();
	}else if(green.y > 140){
		cbc_straight(1000, extra = -20);
		msleep(150);
		update_green();
		printpos();
	}

	if(green.x < 70){
		cbc_spin(1000, extradeg = get_equation(choose_equation()) - 5 );
		msleep(150);
		update_green();
		printpos();
	}else if(green.x > 140){
		cbc_spin(1000, extradeg = get_equation(choose_equation()) + 5);
		msleep(150);
		update_green();
		printpos();
	}

	cbc_spin(1000, deg = get_equation(choose_equation()) );

	//retest it 
	/*
	update_green();
	if(green.x > 166 || green.x < 120)
		cbc_spin(1000, deg = get_equation(choose_equation()) );
	*/

	msleep(150);
	update_green();
	printpos();

	cbc_straight(1000, dist = get_ydist());

	grab_it();

	cbc_straight(1000, -dist);
	cbc_spin(1000, -deg);
	if(extradeg != 0)
		cbc_spin(1000, -extradeg);
	if(extra != 0)
		cbc_straight(1000, -extra);

	stopmotors();
	thread wigglet = thread_create(&claw_wiggle);
	thread wiggleb = thread_create(&basket_wiggle);
	thread_start(wigglet);
	thread_start(wiggleb);
}

int update_green(){
	green = get_object_centroid(GRN, 0);
	int area = get_object_area(GRN, 0);
	int i;
	for(i = 1; i < get_object_count(GRN); i++){
		if(get_object_area(GRN, i) > area){
			area = get_object_area(GRN, i);
			green = get_object_centroid(GRN, i);
		}
	}
	return area;
}	

int update_orange(void){
	orange = get_object_centroid(ORNG, 0);
	int area = get_object_area(ORNG, 0);
	int i;
	for(i = 1; i < get_object_count(ORNG); i++){
		if(get_object_area(ORNG, i) > area){
			area = get_object_area(ORNG, i);
			orange = get_object_centroid(ORNG, i);
		}
	}
	return area;
}	

void printpos(void){
	printf("x:%d y:%d\n", green.x, green.y);
}

int get_ydist(void){
	//return green.y > 15 ? green.y * -.421 + 30.4941 : green.y * -.607142 + 41.714;
	//return green.y > 24 ? green.y * -.3517 + 37.747 : green.y * -.9966 + 52.55; //new equation
	return green.y > 24 ? green.y * -.2883 + 25.565 : green.y * -1.0363 + 46.34;
}

int get_xdeg(void){
	update_green();
	/*
	if(green.x < 40) //left side
		return green.y * -.1178 - 14.9;
	else if(green.x > 280) //right side
		return .00012 * (green.y * green.x) + 14.2; 
	else //middle
		return -19.5 + .1066 * green.x;
	*/

	return green.x * .1119 - 14.49;
}

//stops motors when it finds a green above coords if pos, or below if neg
int find_green(int xcood, int ycood, int delayms){
	msleep(delayms);

	int xmult, ymult;
	xmult = ymult = 1;

	if(xcood < 0){
		xmult = -1;
		//xcood = -xcood;
	}
	if(ycood < 0){
		ymult = -1;
		//ycood = -ycood;
	}

	while(1){
		//camera_update();
		update_green();
		if(green.x != -1 && green.x * xmult > xcood && green.y * ymult > ycood){
			stopmotors();
			return;
		}
	}
}

int update_obj(void){
	const int color = get_object_count(GRN) == 0 ? ORNG : GRN;
	
	myobj = get_object_centroid(color, 0);
	int area = get_object_area(color, 0);
	int i;
	for(i = 1; i < get_object_count(color); i++){
		if(get_object_area(color, i) > area){
			area = get_object_area(color, i);
			myobj = get_object_centroid(color, i);
		}
	}
	return area;
}	

void testfuncs(int argc){
	while(argc == 2){//also search test
		search();
		printf("press side button!\n");
		while(!side_button());
		while(side_button());
	}

	while(argc == 3){//full search test
		set_servo_position(GRAB, DOWN);
		while(!side_button());
		set_servo_position(GRAB, DROP);
		while(side_button());

		checkpos;

		int deg = get_xdeg();
		printf("recommend deg of %d\n", deg);

		point2 mypos = green;

		cbc_spin(1000, deg);

		update_green();
		printpos();

		checkpos;

		int dist = get_ydist();
		cbc_straight(1000, dist);

		grab_it();

		cbc_straight(1000, -dist);
		cbc_spin(1000, -deg);

		printf("X:%d Y:%d\n", mypos.x, mypos.y);

		sleep(2);
	}

	int i = 0;
	while(argc == 4){//manual dist entry
		set_servo_position(GRAB, DOWN);
		while(!side_button());
		set_servo_position(GRAB, DROP);
		while(side_button());

		checkpos;

		point2 mypos = green;

		printf("recomend: %d\n", get_ydist());
		
		printf("DEGS=%d\n", i);
		while(!side_button()){
			if(a_button()){
				i += 2;
				printf("DEGS=%d\n", i);
				while(a_button());
			}
			if(b_button()){
				i -= 2;
				printf("DEGS=%d\n", i);
				while(b_button());
			}
		}
		while(side_button());

		sleep(1);
		cbc_straight(1000, i);

		//checkpos;

		//cbc_straight(1000, get_ydist());

		grab_it();
		printf("DEGS=%d\n", i);
		printf("X:%d Y:%d\n", mypos.x, mypos.y);
		sleep(2);
	}

	while(argc == 5){//manual turning deg entry
		set_servo_position(GRAB, DOWN);
		while(!side_button());
		set_servo_position(GRAB, DROP);
		while(side_button());

		checkpos;

		point2 mypos = green;

		printf("recomend: %d\n", get_equation(choose_equation()) );
		
		printf("DEGS=%d\n", i);
		while(!side_button()){
			if(a_button()){
				i += 2;
				printf("DEGS=%d\n", i);
				while(a_button());
			}
			if(b_button()){
				i -= 2;
				printf("DEGS=%d\n", i);
				while(b_button());
			}
		}
		while(side_button());

		sleep(1);
		cbc_spin(1000, i);

		checkpos;

		cbc_straight(1000, get_ydist());

		grab_it();
		printf("DEGS=%d\n", i);
		printf("X:%d Y:%d\n", mypos.x, mypos.y);
		sleep(2);
	}

	//choose equations
	while(argc == 6){ 
		set_servo_position(GRAB, DOWN);
		while(!side_button());
		set_servo_position(GRAB, DROP);
		while(side_button());

		checkpos;

		point2 mypos = green;
		
		printf("Reccomend equation %d\n", choose_equation());
		
		printf("i=%d\n", i);
		while(!side_button()){
			if(a_button()){
				i += 1;
				printf("i=%d\n", i);
				while(a_button());
			}
			if(b_button()){
				i -= 1;
				printf("i=%d\n", i);
				while(b_button());
			}
		}
		while(side_button());

		sleep(1);
		cbc_spin(1000, get_equation(i));

		checkpos;

		cbc_straight(1000, get_ydist());

		grab_it();
		printf("i=%d\n", i);
		printf("X:%d Y:%d\n", mypos.x, mypos.y);
		sleep(2);
	}
}

