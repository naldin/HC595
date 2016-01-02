/////////////////////////////////////////////////////////////////////////////////////////////////////
// Serial e HC595 v3                                                                               //
// Desenvolvido por Ronaldo R. Jr                                                                  //
// 12/2015                                                                                         //
// naldin.net at gmail                                                                             //
// Referências:                                                                                    //
// Livro Make: AVR Programming, Elliot Williams                                                    //
// Web:                                                                                            //
// http://extremeelectronics.co.in/avr-tutorials/using-shift-registers-with-avr-micro-avr-tutorial //
/////////////////////////////////////////////////////////////////////////////////////////////////////

// ATMega328p trabalhando em 1Mhz

#include <avr/io.h>
#include <util/delay.h>
#include <util/setbaud.h>
#include <avr/interrupt.h>

#define BAUD  9600               // setar o valor desejado

#define HC595_PORT   PORTB       // Definindo portas B
#define HC595_DDR    DDRB        // Definindo direção das portas

#define HC595_DS_POS PB0         //Pino de dados (DS)
#define HC595_SH_CP_POS PB1      // Pino do Clock (SH_CP)
                                 // O clock está em PB1 setado pelo Timer1 modo CTC para OC1A
#define HC595_ST_CP_POS PB2      //Pino do Latch (ST_CP)


// Definindo variaveis
uint8_t serialCaractere = 0xFF;
uint8_t comparador = 0;
int contador = 0; 

// Configurando o clock e interrupção
void ligaClock(void){
	TCCR1A |= (1 << COM1A0);             // Modo alternado
	TCCR1B |= (1 << WGM12);              // Modo CTC em OC1A (PB1)
	TCCR1B |= (1 << CS10);               // Prescaler = 1
	TIMSK1 |= (1 << OCIE1A);             // Habilita interrupcao de saida por comparacao
	
	sei(); // Habilita interrupção
}

/*** Inicio tratamento serial ***/

// Setup da serial
void iniciaUSART(void) {             // Necessario setar BAUD acima
  UBRR0H = UBRRH_VALUE;              // Ja esta definido em setbaud.h
  UBRR0L = UBRRL_VALUE;              // Ja esta definido em setbaud.h

// Habilita TX e RX da USART
  UCSR0B = (1 << TXEN0) | (1 << RXEN0);
  UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);   // 8 bits de dados, 1 bit de parada

  #if USE_2X
    UCSR0A |= (1 << U2X0);
  #else
    UCSR0A &= ~(1 << U2X0);
  #endif
}

void enviaByte(uint8_t dado) {
  loop_until_bit_is_set(UCSR0A, UDRE0);      // Aguardar bit para enviar
  UDR0 = dado;                               // Envia dado
}

uint8_t recebeByte(void) {
  loop_until_bit_is_set(UCSR0A, RXC0);       // Aguarda chegada de dados
  return UDR0;                               // Retorna o dado em recebeByte
}

void imprimeString(const char valorS[]) {    // Imprime uma palavra ou letra
  uint8_t i = 0;
  while (valorS[i]) {
    enviaByte(valorS[i]);
    i++;
  }
}

// Recebe na serial um numero que tenha mais de um caracter e concatena.
// A serial envia apenas um caracter por vez, portanto para enviar 123 é enviado 1, depois 2, depois 3.
// Esta função concatena os numeros e retona em um byte (uint8_t)
uint8_t recebeNumero(void) {
  char centenas = '0';
  char dezenas = '0';
  char unidades = '0';
  char valorAtual = '0';
  do {
    centenas = dezenas;
    dezenas = unidades;
    unidades = valorAtual;
    valorAtual = recebeByte();
    enviaByte(valorAtual);
  } while (valorAtual != '\r');   // aguarda o enter para continuar
  return (100 * (centenas - '0') + 10 * (dezenas - '0') + unidades - '0');
}

// Envia para serial o numero em decimal.
// Para visualizar na serial um numero enviado 123 (ex.), é necessario separar cada caracter,
// se for enviado o byte 123 a serial recebera o caracter { que é  o valor ascii convertido.
void printByte(uint8_t byte) {           // Converte um byte em um texto decimal
  enviaByte('0' + (byte / 100));         // Centenas
  enviaByte('0' + ((byte / 10) % 10));   // Dezenas
  enviaByte('0' + (byte % 10));          // Unidades
}

/*** Fim tratamento serial ***/
//=========================================================================//
/*** Inicio tratamento HC595 ***/

// Setando saidas para Data(DS), Clock (SH_CP), Latch (ST_CP)
void PortaOn(void) {
   DDRB |= (1 << HC595_SH_CP_POS) | (1 << HC595_ST_CP_POS) | (1 << HC595_DS_POS);
}

// Desabilitando portas
void PortaOff(void) {
   DDRB &= ~ (1 << HC595_SH_CP_POS) | (1 << HC595_ST_CP_POS) | (1 << HC595_DS_POS);
}

// Enviando 8 bits serialmente
void BitsHC595(void){
	if(contador%2 == 0) {   // Somente envia os bits no contador de valor par. Na subida dos pulsos de clock
		if (bit_is_set(serialCaractere, 7)){   // Se o oitavo (0 a 7) bit for 1 saida = high (1)
		  
			  HC595_PORT |= (1 << HC595_DS_POS);
		  }
		  else {                  // Senao saida = low (0)
		  
			  HC595_PORT &= ~(1 << HC595_DS_POS);
		  }
		  serialCaractere = serialCaractere << 1;  // Move o bit em direção ao MSB
	 }
	  
	 if (contador == 16){   // No pulso 16 do clock coloca a porta em 1 para o latch
		  HC595_PORT |= (1 << HC595_ST_CP_POS);
	 }
	 if (contador == 17){   // No pulso 17 do clock coloca a porta em 0 para o latch
		  HC595_PORT &= ~(1 << HC595_ST_CP_POS);
		  PortaOff();        // Desliga todas as portas
		  comparador = serialCaractere;   // Iguala comparador com valor atual e para interrupcoes
	 }
	 contador++;
}

// Envia 1 bit em cada interrupção
ISR(TIMER1_COMPA_vect) {
	  BitsHC595();
}

/*** Fim tratamento HC595 ***/

int main(void) {
	ligaClock();           // Inicializa o clock para HC595
	iniciaUSART();         // Inicializa serial
	imprimeString("Digite: um valor entre 0 e ");
	
	while(1){
		if (comparador != serialCaractere){  // Compara valor digitado com valor anterior
			PortaOn();                       // Habilita as portas
			OCR1A = 200;                     // Define frequencia do clock para HC 595. Em 200 periodo de 400uS freq 2500hz.
			contador = 0;
		}
		printByte(serialCaractere);          // Imprime numero
		imprimeString("\n");                 // Nova linha
		serialCaractere = recebeNumero();    // Aguarda novo numero e grava em serialCaractere
	}
}
