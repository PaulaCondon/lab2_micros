#define F_CPU 16000000UL
#include <avr/io.h>
#include <util/delay.h>
#include <avr/eeprom.h>
#define EEPROM_ADDRESS 0x00
#define  PIN_motor1_A PORTB1
#define  PIN_motor1_B PORTB2
#define  PIN_motor2_A PORTB3
#define  PIN_motor2_B PORTB4
#define PIN_cambiar_velocidad PB0
#define PIN_pulsador_apagar PB5
#define PIN_pulsador_colisiones PD7
#define PIN_led_blanca PORTD1
#define PIN_led_verde PORTD2
#define PIN_led_rojo PORTD4
#define TiempoCambioVelocidad_ms 3
enum EstadosMotores{apagado,adelante,adelanteV2,adelanteV3, atras, izquierda, derecha, CANT_ESTADOS};

uint8_t estadoActual = 0;
uint16_t colisiones = 0;  // Valor conteo de colisiones

void initConteoColisiones();					//inicializa conteo de activaciones
void initPines();								//inicializa pines(entradas - salidas)
void ADC_init();								//incializa adc
uint16_t ADC_read(uint8_t channel);				//lectura de adc(canal seleccionado ADC)
void PWM_init();								//inicializa pwm
void set_motores(int velocidad, int sentido);	//cambia la velocidad y sentido de los motores(velocidad(0 - 4) y sentido(0 = adelante, 1 = atras, 2 = derecha, 3 = izquierda))
//funciones utilizadas para cada estado de los motores, que modifican la velocidad y sentido
void Apagado();
void Adelante();
void AdelanteV2();
void AdelanteV3();
void Atras();
void Izquierda();
void Derecha();

int main(void){
	//inicializa pines
	initPines();
	//inicializa adc
	ADC_init();
	uint16_t adc_value;
	//inicializa pwm
	PWM_init();
	//setea velocidad y sentido inicial(motores apagados)
	estadoActual = apagado;
	void (*vector_estados[CANT_ESTADOS])();
	vector_estados[apagado] = Apagado;
	vector_estados[adelante] = Adelante;
	vector_estados[adelanteV2] = AdelanteV2;
	vector_estados[adelanteV3] = AdelanteV3;
	vector_estados[atras] = Atras;
	vector_estados[izquierda] = Izquierda;
	vector_estados[derecha] = Derecha;	
	
	while(1){
		if((PINB & (1 << PIN_pulsador_apagar)) == 0){
			//estados motores
			vector_estados[estadoActual]();
			_delay_ms(10);
			//lee el valor y lo convierte desde potenciometro PC0
			adc_value = ADC_read(0);
			//si hay poca luz enciende luz blanca, sino la apaga
			if (adc_value < 550) {
				PORTD |= (1 << PIN_led_blanca);  // Enciende el LED
				} else {
				PORTD &= ~(1 << PIN_led_blanca); // Apaga el LED
			}
			if (PIND & (1 << PIN_pulsador_colisiones)){
				colisiones = eeprom_read_word((uint16_t*)EEPROM_ADDRESS);	//carga valor guardado en la EEPROM_ADDRESS(0x00)
				colisiones++;												//le suma 1
				eeprom_write_word((uint16_t*)EEPROM_ADDRESS, colisiones);	//guarda el valor de la variable conteo en la eeprom en la direccion EEPROM_ADDRESS(0x00)
				_delay_ms(500);
			}
		}else{
			estadoActual = apagado;
			PORTD &= ~(1 << PIN_led_blanca); // Apaga el LED											
			_delay_ms(50);
			}
	}
	return 1;
}

void initConteoColisiones(){
	colisiones = 0;
	eeprom_write_word((uint16_t*)EEPROM_ADDRESS, colisiones);//inica el valor 0 de conteo en la eeprom en la direccion EEPROM_ADDRESS(0x00)
}

void initPines(){
		/*PINES:
	
	pd 1 led blanca		PD1 1-
	pd 2 led verde		PD2 1-
    pd 13 pulsador		PB5 1				
	pd 4 led rojo		PD4 1-
	pd 5 pwm1			PD5 1 //velocidad -
	pd 6 pwm2			PD6 1-
	pd 7 choque			PD7 0-
	8  CAMBIO VELOCIDAD PB0
	pd 9-10 motor 1		PB1 Y PB2 //giro motor1
	pd 11-12 motor 2	PB3 Y PB4	giro motor2			*/
		
	//entradas PORTD
	DDRD &= ~((1 << PD7));
	//salidas PORTD
	DDRD |= (1 << DDD1)|(1 << DDD2)|(1 << DDD4)|(1 << DDD5)|(1 << DDD6);
	PORTD |= (1 << PIN_led_blanca);//ENCIENDE LUZ BLANCA
	
	//entradas PORTC
	DDRC &= ~(1 << PINC0);
	
	//entradas PORTB
	DDRB &= ~((1 << PIN_cambiar_velocidad)|(1 << PIN_pulsador_apagar));
	//salidas PORTB
	DDRB |= (1 << DDB1)|(1 << DDB2)|(1 << DDB3)|(1 << DDB4);
	//pull down
	PORTB &= ~((1 << PORTB5)|(1 << PORTB0));
}


void ADC_init() {
	// Configura la referencia de voltaje a AVCC
	ADMUX = (1 << REFS0);

	// Habilita el ADC y establece el prescaler a 128 para un reloj de 16 MHz (125 kHz)
	ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
}

uint16_t ADC_read(uint8_t channel) {
	// Configura el canal de entrada analógica
	ADMUX = (ADMUX & 0xF8) | (channel & 0x07);

	// Inicia la conversión
	ADCSRA |= (1 << ADSC);

	// Espera a que la conversión se complete
	while (ADCSRA & (1 << ADSC));

	// Lee y devuelve el valor del ADC
	return ADC;
}

void PWM_init() {
	// Configuracion de los pines para PWM (TOP en 0xFF)
	TCCR0A |= (1 << WGM00) | (1 << WGM01)| (1 << COM0A1) | (1 << COM0B1);
	TCCR0B |= (1 << CS00); // prescaler 1
}

//setea velocidad(0 - 4) y sentido de los motores(0 = adelante, 1 = atras, 2 = izquierda, 3 = derecha)
void set_motores(int velocidad, int sentido){
	switch(sentido){
		case 0:
			PORTB |= (1 << PIN_motor1_A)|(1 << PIN_motor2_A);	//adelante
			PORTB &= ~((1 << PIN_motor1_B)|(1 << PIN_motor2_B));
			PORTD |= (1 << PIN_led_verde);										//enciende led verde
			PORTD &= ~(1 << PIN_led_rojo);										//apaga led rojo
			break;
		case 1:
			PORTB &= ~ ((1 << PIN_motor1_A)|(1 << PIN_motor2_A));	//ATRAS
			PORTB |= (1 << PIN_motor1_B)|(1 << PIN_motor2_B);
			PORTD &= ~(1 << PIN_led_verde);										//apaga led verde
			PORTD |= (1 << PIN_led_rojo);										//enciende led rojo
			break;
		case 3:
			PORTB |= (1 << PIN_motor1_A);	//izquierda
			PORTB &= ~((1 << PIN_motor1_B)|(1 << PIN_motor2_B)|(1 << PIN_motor2_A));
			PORTD |= (1 << PIN_led_verde);										//enciende led verde
			PORTD &= ~(1 << PIN_led_rojo);										//apaga led rojo
			break;
		default:
			PORTB &= ~ ((1 << PIN_motor1_A)|(1 << PIN_motor1_B)|(1 << PIN_motor2_B));	//derecha
			PORTB |=(1 << PIN_motor2_A);
			PORTD |= (1 << PIN_led_verde);										//enciende led verde
			PORTD &= ~(1 << PIN_led_rojo);										//apaga led rojo
			break;
	}
	switch(velocidad){
		case 0:
			OCR0A = 0;
			OCR0B = 0;
			//si es cero la velocidad apaga ambos leds
			PORTD &= ~(1 << PIN_led_verde);	//apaga led verde
			PORTD &= ~(1 << PIN_led_rojo);	//apaga led rojo
			break;
		case 1:
			while(OCR0A > 80){				//si la velocidad actual es mayor que 85 le resta
				OCR0A--;
				OCR0B--;
				_delay_ms(TiempoCambioVelocidad_ms);
			}
			while(OCR0A < 80){				//si la velocidad actual es mayor que 85 le suma
				OCR0A++;
				OCR0B++;
				_delay_ms(TiempoCambioVelocidad_ms);
			}
			break;
		case 2:
			while(OCR0A > 150){				//si la velocidad actual es mayor que 170 le resta
				OCR0A--;
				OCR0B--;
				_delay_ms(TiempoCambioVelocidad_ms);
			}
			while(OCR0A < 150){				//si la velocidad actual es mayor que 170 le suma
				OCR0A++;
				OCR0B++;
				_delay_ms(TiempoCambioVelocidad_ms);
			}
			break;
		default:
			while(OCR0A > 254){				//si la velocidad actual es mayor que 254 le resta(no deberia pasar)
				OCR0A--;
				OCR0B--;
				_delay_ms(TiempoCambioVelocidad_ms);
			}
			while(OCR0A < 254){				//si la velocidad actual es mayor que 254 le suma
				OCR0A++;
				OCR0B++;
				_delay_ms(TiempoCambioVelocidad_ms);
			}
			break;
	}
}

void Apagado(){
	set_motores(0, 0);
	estadoActual = apagado;
	if(PINB & (1 << PIN_cambiar_velocidad)){
		estadoActual = adelante;
		_delay_ms(500);
	}
}

void Adelante(){
	set_motores(1, 0);
	estadoActual = adelante;
	if(PINB & (1 << PIN_cambiar_velocidad)){
		estadoActual = adelanteV2;
		_delay_ms(500);
	}
}

void AdelanteV2(){
	set_motores(2, 0);
	estadoActual = adelanteV2;
	if(PINB & (1 << PIN_cambiar_velocidad)){
		estadoActual = adelanteV3;
		_delay_ms(500);
	}
}

void AdelanteV3(){
	set_motores(3, 0);
	estadoActual = adelanteV3;
	if(PINB & (1 << PIN_cambiar_velocidad)){
		estadoActual = atras;
		_delay_ms(500);
	}
}

void Atras(){
	set_motores(1,1);
	estadoActual = atras;
	if(PINB & (1 << PIN_cambiar_velocidad)){
		estadoActual = izquierda;
		_delay_ms(500);
	}
}

void Izquierda(){
	set_motores(1,2);
	estadoActual = izquierda;
	if(PINB & (1 << PIN_cambiar_velocidad)){
		estadoActual = derecha;
		_delay_ms(500);
	}
}

void Derecha(){
	set_motores(1,3);
	estadoActual = derecha;
	if(PINB & (1 << PIN_cambiar_velocidad)){
		estadoActual = apagado;
		_delay_ms(500);
	}
}