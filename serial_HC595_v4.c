/////////////////////////////////////////////////////////////////////////////////////////////////////
// Serial e HC595 v4                                                                               //
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

// Inicializando SPI
void iniciaSPI(void) {
  DDRB |= (1 << PB1);         // Setando saida para Latch (ST_CP)
  PORTB &= ~(1 << PB1);       // Setando porta do Latch (ST_CP) para low (0)
 
  DDRB |= (1 << PB2);         // Setando saida para SS do SPI, não usado mas necessario para SPI funcionar
  PORTB |= (1 << PB2);        // Setando porta SS para high (1) do SPI, não usado mas necessario para SPI funcionar
 
  DDRB |= (1 << PB3);         // Setando saida Dados (DS) - MOSI no SPI
  PORTB |= (1 << PB3);        // Setando porta de Dados para high (1)
  DDRB |= (1 << PB5);         // Setando saida para o Clock (SH_CP)

  SPCR |= (1 << SPR1);        // Divide frequencia 1Mhz por 64 = 15625hz
  SPCR |= (1 << MSTR);        // Define o AVR como Master no SPI
  SPCR |= (1 << SPE);         // Habilita SPI
  SPCR |= (1 << CPOL);        // Inicializa clock em high (1)
  SPCR |= (1 << CPHA);        // Inicializa dados MOSI na descida do clock
}

// Funcao de latch
void latchHC595(void){
	PORTB |= (1 << PB1);
	_delay_us(65);        // Caso nao use capacitor no latch para o terra o delay pode ser removido
	PORTB &= ~(1 << PB1);
}

// Envia byte pela SPI
void SPI_enviaByte(uint8_t byte) {
  SPDR = byte;                       /* SPI starts sending immediately */
  loop_until_bit_is_set(SPSR, SPIF);                /* wait until done */
                                /* SPDR now contains the received byte */
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

int main(void) {
	uint8_t serialCaractere = 0xFF;  // Variavel
	iniciaUSART();           // Inicializa serial
	iniciaSPI();             // Inicializa SPI
	imprimeString("Digite um valor entre 0 e 255:\r\n");
	
	while(1){
		
		SPI_enviaByte(serialCaractere);        // Envia o byte
		latchHC595();                          // Envia o Latch
		serialCaractere = recebeNumero();      // Aguarda novo numero e grava em serialCaractere
		imprimeString("\r\n");                 // Nova linha
		printByte(serialCaractere);            // Imprime numero
		imprimeString("\r\n");                 // Nova linha
		
	}
}
