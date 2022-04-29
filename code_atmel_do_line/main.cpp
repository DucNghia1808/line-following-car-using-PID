#define F_CPU 8000000UL
#include <avr/io.h>
#include <avr/delay.h>
#include <stdio.h>
#include <avr/interrupt.h>

#define tbi(port,bit){port^=(1<<bit);} // dao bit
#define sbi(port,bit){port|=(1<<bit);}
#define cbi(port,bit){port&=~(1<<bit);}
#define motor_speed 20000  // v tb set OCR // 166
#define db 150
unsigned char k=0;
 
float Kp = 40;
float Ki = 0;
float Kd = 15;

float error = 0, P = 0, I = 0, D = 0, PID_value = 0;
float prev_error = 0, prev_I = 0;

void send(unsigned char c){        // ham gui ki tu
	while(bit_is_clear(UCSRA,UDRE)); // doi den khi UDRE =1
	UDR=c;
}
void send_string(char *string){        // ham gui chuoi ki tu
	for(int i=0;i<255;i++){
		if(string[i]!=0){
			send(string[i]);
		}
		else break;
	}
}

void set_OCR1A(signed long x){ // doi OCR1A  motor 1 (LEFT)
	if(x>255)x=255;
	if(x<0)x=0;
	unsigned int widthA;
	widthA=(int)((x*motor_speed)/255);
	OCR1A=widthA;
}

void set_OCR1B(signed long x){ // doi OCR1B  motor 2 (RIGHT)
	if(x>255)x=255;
	if(x<0)x=0;
	unsigned int widthB;
	widthB=(int)((x*motor_speed)/255);
	OCR1B=widthB;
}

void caculate_PID(){
	P=error;
	I=I+prev_I;
	D=error-prev_error;
	PID_value = Kp*P+Ki*I+Kd*D;
	prev_I=I;
	prev_error=error;
}

void forward(){
	sbi(PORTB,0); // bit 1             motor 1 
	cbi(PORTB,1); // bit 0                  
	sbi(PORTB,2); // bit 1             motor 2
	cbi(PORTB,3); // bit 0                  
}

void reverse(){
	cbi(PORTB,0); // bit 1             motor 1
	sbi(PORTB,1); // bit 0
	cbi(PORTB,2); // bit 1             motor 2
	sbi(PORTB,3); // bit 0
}

void left(){ // code du phong sang trai
	cbi(PORTB,0); // bit 1             motor 1
	cbi(PORTB,1); // bit 0
	sbi(PORTB,2); // bit 1             motor 2
	cbi(PORTB,3); // bit 0
}

void right(){ // code du phong sang phai
	sbi(PORTB,0); // bit 1             motor 1
	cbi(PORTB,1); // bit 0
	cbi(PORTB,2); // bit 1             motor 2
	cbi(PORTB,3); // bit 0
}

void rorate_left(){ // vuong trai
	cbi(PORTB,0); // bit 1             motor 1 (LEFT)
	sbi(PORTB,1); // bit 0
	sbi(PORTB,2); // bit 1             motor 2  (RIGHT)
	cbi(PORTB,3); // bit 0
}

void rorate_right(){
	sbi(PORTB,0); // bit 1             motor 1 (LEFT)
	cbi(PORTB,1); // bit 0
	cbi(PORTB,2); // bit 1             motor 2  (RIGHT)
	sbi(PORTB,3); // bit 0
}

void stop(){
	cbi(PORTB,0); // bit 1             motor 1 (LEFT)
	cbi(PORTB,1); // bit 0
	cbi(PORTB,2); // bit 1             motor 2  (RIGHT)
	cbi(PORTB,3); // bit 0
}

void read_sensor(){
	if((bit_is_set(PINA,0))&&(bit_is_set(PINA,1))&&(bit_is_clear(PINA,2))&&(bit_is_set(PINA,3))&&(bit_is_set(PINA,4))) {
		error=0; // forward
		forward();
	}

	else if((bit_is_set(PINA,0))&&(bit_is_clear(PINA,1))&&(bit_is_clear(PINA,2))&&(bit_is_set(PINA,3))&&(bit_is_set(PINA,4))){
		error=-1;  //left 1
		right();
	}
	
	else if((bit_is_set(PINA,0))&&(bit_is_set(PINA,1))&&(bit_is_clear(PINA,2))&&(bit_is_clear(PINA,3))&&(bit_is_set(PINA,4))){
		error=1; // right 1
		left();
	}

	else if((bit_is_set(PINA,0))&&(bit_is_clear(PINA,1))&&(bit_is_set(PINA,2))&&(bit_is_set(PINA,3))&&(bit_is_set(PINA,4))){
		error=-2;// left 2
		right();
	}
	else if((bit_is_set(PINA,0))&&(bit_is_set(PINA,1))&&(bit_is_set(PINA,2))&&(bit_is_clear(PINA,3))&&(bit_is_set(PINA,4))){
		error=2;  // right 2
		left();
	}

	else if((bit_is_clear(PINA,0))&&(bit_is_clear(PINA,1))&&(bit_is_set(PINA,2))&&(bit_is_set(PINA,3))&&(bit_is_set(PINA,4))){
		error=-3; //left 3
		right();
	}
	
	else if((bit_is_set(PINA,0))&&(bit_is_set(PINA,1))&&(bit_is_set(PINA,2))&&(bit_is_clear(PINA,3))&&(bit_is_clear(PINA,4))){
		error=3;// right 3
		left();
	}

	else if((bit_is_clear(PINA,0))&&(bit_is_set(PINA,1))&&(bit_is_set(PINA,2))&&(bit_is_set(PINA,3))&&(bit_is_set(PINA,4))){
		error=-4; //left 4
		right();
	}
	
	else if((bit_is_set(PINA,0))&&(bit_is_set(PINA,1))&&(bit_is_set(PINA,2))&&(bit_is_set(PINA,3))&&(bit_is_clear(PINA,4))){
		error=4;// right 4
		left();
	}
	
	else if((bit_is_clear(PINA,0))&&(bit_is_clear(PINA,1))&&(bit_is_clear(PINA,2))&&(bit_is_set(PINA,3))&&(bit_is_set(PINA,4))){
		error=10; //reverse left
		//rorate_left();
	}

	else if((bit_is_set(PINA,0))&&(bit_is_set(PINA,1))&&(bit_is_clear(PINA,2))&&(bit_is_clear(PINA,3))&&(bit_is_clear(PINA,4))){
		error=11;// reverse right
		//rorate_right();
	}
	
	else if((bit_is_set(PINA,0))&&(bit_is_set(PINA,1))&&(bit_is_set(PINA,2))&&(bit_is_set(PINA,3))&&(bit_is_set(PINA,4))){
		error=12; // out line all                                   (...)or( stop) or (net dut);
	}
	else if((bit_is_clear(PINA,0))&&(bit_is_clear(PINA,1))&&(bit_is_clear(PINA,2))&&(bit_is_clear(PINA,3))&&(bit_is_clear(PINA,4))){
		error=13; // in line all                                    (+) then (stop)
	}
}

void d_b(){
	if(error==10){//// xoay trai
		do{
			set_OCR1A(db);
			set_OCR1B(db);
			rorate_right();
			read_sensor();
		}while((error!=0)&&bit_is_clear(PINA,5));
	}
	else if(error==11){    // xoay phai
		do{
			set_OCR1A(db);
			set_OCR1B(db);
			rorate_left();
			read_sensor();
		}while((error!=0)&&bit_is_clear(PINA,5));
	}
	else if(error==12){
		if(bit_is_clear(PINA,5)){     // net dutttttttttttttt
			forward();
			set_OCR1A(db);
			set_OCR1B(db);
			_delay_ms(600);    // doi cho qua net dut
		}
		else if(bit_is_set(PINA,5)){ /// vang ALL chua biet lam gi
			do{
				set_OCR1A(db); // vang all xoay trai
				set_OCR1B(db);
				rorate_left();
				read_sensor();
			}while(error!=0);
		}
	}
	else if(error==13){ // (+) or (dung)
		if(bit_is_clear(PINA,5)){ ///++++++++++++++++++++++++++++++++++
			forward();                    // cho chay toi mot doan
			set_OCR1A(db);
			set_OCR1B(db);
			_delay_ms(500); // can cho qua dau ++ khong qua vung den
			if((bit_is_clear(PINA,0))&&(bit_is_clear(PINA,1))&&(bit_is_clear(PINA,2))&&(bit_is_clear(PINA,3))&&(bit_is_clear(PINA,4))&&(bit_is_clear(PINA,5))){
				stop(); /// vao vung dden hoan toan dung lai
				//_delay_ms(25000);
			}
		}
	}
}

ISR(INT0_vect){
	k++;
	if(k==3)
		k=0;
} 

void dieu_chinh(){
	do{
		if(bit_is_clear(PINA,6)){
			while(bit_is_clear(PINA,6));
			Kp++;
		}
		if(bit_is_clear(PINA,7)){
			while(bit_is_clear(PINA,7));
			Kp--;
		}
	}while(bit_is_set(PINB,4));// kiem tra dk lap neu dung
	do{
		if(bit_is_clear(PINA,6)){
			while(bit_is_clear(PINA,6));
			Ki++;
		}
		if(bit_is_clear(PINA,7)){
			while(bit_is_clear(PINA,7));
			Ki--;
			if(Ki<0)Ki=0;
		}
	}while(bit_is_set(PINB,4));
	do{
		if(bit_is_clear(PINA,6)){
			while(bit_is_clear(PINA,6));
			Kd++;
		}
		if(bit_is_clear(PINA,7)){
			while(bit_is_clear(PINA,7));
			Kd--;
		}
	}while(bit_is_set(PINB,4));
	k=0;// ve ban dau
}
int main(void)
{	k=0;
	DDRA=0x00;// connect to sensor
	//PORTA=0xFF;
	DDRB=0xFF; // connect to motor
	PORTB=0x00;
	DDRD=0xFF;
	cbi(DDRD,2);// set chan ngat ngoai
	sbi(PORTD,2);
	cbi(DDRB,4);// set chan mode dieu chinh
	sbi(PORTB,4);

	char tem1[20];
    /* TIMER PWM MODE */
	TCCR1A=(1<<COM1A1) | (1<<COM1B1) | (1<<WGM11);    //chon mode top la ICR1
	TCCR1B=(1<<WGM13) | (1<<WGM12) | (1<<CS11) ;       // bo chia 8  non inverting mode
	ICR1H=0x4E; // 1 chu ki xung la 20 ms <=> 20000=0x4E20
	ICR1L=0x20;     // tong o ICRH va ICRL la 20000
	// INT0: On    Low level
	GICR|=(1<<INT0); // bat int
	MCUCR|=(1<<ISC10);  // che do hoat dong int0
	sei();
    while (1) 
    {	
		/*if(k==1){
				
		}
		else if(k==2){ // dieu chinh thong so
			dieu_chinh();
		}*/
		read_sensor();
		caculate_PID();
		d_b();
		set_OCR1B(110-PID_value);// so ma thang line chay
		set_OCR1A(110+PID_value);
		//forward();
	}
}

