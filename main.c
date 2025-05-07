#define F_CPU 16000000UL
#include <avr/io.h>
#include <util/delay.h>
#include <stdint.h>
#include <avr/interrupt.h>
#include <math.h>

//#define DIR_TX() (PORTC |=  (1 << PC5))
//#define DIR_RX() (PORTC &= ~(1 << PC5))
#define DIR_TX() (PORTC |= 0x20)
#define DIR_RX() (PORTC &= 0xdf)

void uart_init() {
  uint16_t ubrr = 103;  // 9600bps
  UBRRH = (ubrr >> 8);
  UBRRL = ubrr;
  UCSRB = (1 << RXEN) | (1 << TXEN);
  UCSRC = (1 << URSEL) | (3 << UCSZ0); // 8-bit
}

void uart_send(uint8_t data) {
  while (!(UCSRA & (1 << UDRE)));
  UDR = data;
}

uint8_t uart_recv() {
  while (!(UCSRA & (1 << RXC)));
  return UDR;
}

uint16_t modbus_crc(uint8_t *buf, uint8_t len) {
  uint16_t crc = 0xFFFF;
  for (uint8_t i = 0; i < len; i++) {
    crc ^= buf[i];
    for (uint8_t j = 0; j < 8; j++) {
      if (crc & 1) crc = (crc >> 1) ^ 0xA001;
      else crc >>= 1;
    }
  }
  return crc;
}

float read_torque() {
  uint8_t req[] = {0x01, 0x03, 0x00, 0x00, 0x00, 0x03, 0x05, 0xCB}; // 3레지스터 읽기
  //uint16_t crc = modbus_crc(req, 6);
  //req[6] = crc & 0xFF;
  //req[7] = (crc >> 8) & 0xFF;

  DIR_TX(); _delay_us(20);
  for (uint8_t i = 0; i < 8; i++) uart_send(req[i]);
  _delay_us(1800);
  DIR_RX();

  uint8_t resp[11];
  for (uint8_t i = 0; i < 11; i++) resp[i] = uart_recv();

  // CRC 확인
  uint16_t crc_recv = (resp[10] << 8) | resp[9];
  uint16_t crc_calc = modbus_crc(resp, 9);
  if (crc_calc != crc_recv) return 0;

  uint16_t high = ((uint16_t)resp[3] << 8) | resp[4];
  uint16_t low  = ((uint16_t)resp[5] << 8) | resp[6];
  uint16_t dp   = ((uint16_t)resp[7] << 8) | resp[8];

  int32_t raw = ((int32_t)high << 16) | low;
  float torque = ((float)raw) / pow(10, dp);
  return torque;
}

void send_speed(int16_t rpm) {
  uint8_t dir = (rpm < 0) ? 0x01 : 0x00; // CW if 음수
  if (rpm < 0) rpm = -rpm;
  if (rpm > 65533) rpm = 65533;

  uint8_t pkt[10] = {
    0xFF, 0xFE, 0x00, 0x06, 0x00, 0x03,
    dir,
    (uint8_t)(rpm >> 8), (uint8_t)(rpm & 0xFF),
    0x01  // 도달시간 = 0.1s
  };

  uint8_t sum = 0;
  sum = pkt[2] + pkt[3] + pkt[5] + pkt[6] + pkt[7] + pkt[8] + pkt[9];
  pkt[4] = ~sum;

  DIR_TX(); _delay_us(20);
  for (uint8_t i = 0; i < 10; i++) uart_send(pkt[i]);
  //_delay_us(1800);
  _delay_ms(2);
  DIR_RX();
}

int main(void) {
  //DDRC |= (1 << PC5); // DIR핀 출력 설정
  DDRC = 0x20;
  PORTC = 0x00;
  uart_init();
  _delay_ms(3000);

  while (1) {
    float torque = read_torque();      
    int16_t rpm_cmd = (int16_t)(torque * 70);  
    if (rpm_cmd > 60) rpm_cmd = 60;
    else if (rpm_cmd < -60) rpm_cmd = -60;
    send_speed(rpm_cmd);
    _delay_ms(50);
  }
}
