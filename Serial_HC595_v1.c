/////////////////////////////////////////////////////////////////////////////////////////////////////
// Serial e HC595 v1                                                                               //
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

#define BAUD  9600               // Setar o valor desejado

#define HC595_PORT   PORTB       // Definindo portas B
#define HC595_DDR    DDRB        // Definindo direção das portas

#define HC595_DS_POS PB0         // Pino de dados (DS)
#define HC595_SH_CP_POS PB1      // Pino do Clock (SH_CP)
#define HC595_ST_CP_POS PB2      // Pino do Latch (ST_CP)

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
void HC595Init(void) {
   DDRB |= (1 << HC595_SH_CP_POS) | (1 << HC595_ST_CP_POS) | (1 << HC595_DS_POS);
}

// Envia clock no SH_CP
void HC595Pulse(void) {
   HC595_PORT|=(1<<HC595_SH_CP_POS);    //HIGH
   HC595_PORT&=(~(1<<HC595_SH_CP_POS)); //LOW
}

// Envia latch no ST_CP
void HC595Latch(void) {
   HC595_PORT|=(1<<HC595_ST_CP_POS);    //HIGH
   _delay_us(10);

   HC595_PORT&=(~(1<<HC595_ST_CP_POS)); //LOW
   _delay_us(10);
}

// Enviando 8 bits serialmente
void HC595Write(uint8_t data) {
  
   // Ordenado por MSB
   for(uint8_t i=0;i<8;i++)
   {
      if(data & 0b10000000)  // Se resultado "dados & 0b10000000" for verdadeiro (1) saida = high (1)
      {
         HC595_PORT |= (1 << HC595_DS_POS);
      }
      else                   // Se resultado "dados & 0b10000000" for (0) saida = low (0)
      {
         HC595_PORT &= ~(1 << HC595_DS_POS);
      }

      HC595Pulse();      // Pulso de clock
      data = data << 1;  // Move o bit em direção ao MSB

   }

   // envia o latch 
   HC595Latch();
}

// Função de atraso
void Wait(void) {
	_delay_ms(1000);
   
}

int main(void) {
	
   uint8_t serialCaractere;
   HC595Init();              // Setup da direção das portas
   iniciaUSART();            // Inicializa serial
   HC595Write(0xFF);         // Inicia com valor em 255 (0xFF em HEX)
   imprimeString("Digite: um valor entre 0 e 255\n");
   
   while(1)
   {
	   serialCaractere = recebeNumero();   // Aguarda novo numero e grava em serialCaractere
	   printByte(serialCaractere);         // Imprime numero
	   imprimeString("\n");                // Nova linha
       HC595Write(serialCaractere);        // Envia numero para HC595
       Wait();                             // Aguarda 1s
   }
}
