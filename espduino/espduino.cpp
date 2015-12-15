/**
* \file
*       ESP8266 bridge arduino library
* \author
*       Tuan PM <tuanpm@live.com>
*/
#include "espduino.h"

RESPONSE::RESPONSE(void* response) {
  cmd = (PACKET_CMD*)response;
  arg_ptr = (uint8_t*)&cmd->args;
  arg_num = 0;
}

uint16_t RESPONSE::getArgc() {
  return cmd->argc;
}

uint16_t RESPONSE::argLen() {
  return *(uint16_t*)arg_ptr;
}

int32_t RESPONSE::popArgs(uint8_t* data, uint16_t maxLen) {
  uint16_t length, len, incLen = 0;

  if (arg_num >= cmd->argc)
    return -1;

  length = *(uint16_t*)arg_ptr;
  len = length;
  arg_ptr += 2;

  while (length--) {
    *data++ = *arg_ptr++;
    incLen++;
    if (incLen > maxLen) {
      arg_num++;
      arg_ptr += length;
      return maxLen;
    }

  }
  arg_num++;
  return len;
}

void RESPONSE::popChar(char* buffer) {
  uint16_t len = *(uint16_t*)arg_ptr;
  arg_ptr += 2;
  uint8_t i;
  for (i = 0; i < len; i++) {
    buffer[i] = (char)*arg_ptr++;
  }
  buffer[i] = '\0';
  arg_num++;
}

String RESPONSE::popString() {
  String ret;
  uint16_t len = *(uint16_t*)arg_ptr;
  arg_ptr += 2;
  while (len--)
    ret += (char)*arg_ptr++;
  arg_num++;
  return ret;
}

void RESPONSE::popString(String* data) {
  uint16_t len = *(uint16_t*)arg_ptr;
  arg_ptr += 2;
  while (len--)
    data->concat((char)*arg_ptr++);
  arg_num++;
}

void ESP::protoCompletedCb(void) {
  PACKET_CMD* packet = (PACKET_CMD*)_proto.buf;
  uint16_t crc = 0, argc, len, resp_crc, argn = 0;
  uint8_t* data_ptr;
  argc = packet->argc;
  data_ptr = (uint8_t*)&packet->args;
  crc = crc16_data((uint8_t*)&packet->cmd, 12, crc);

  //  ardprintf(_debug, "ARD CMD: %d, cb: %d, ret: %d, argc: %d", packet->cmd, packet->callback, packet->_return, packet->argc);

  while (argc--) {
    len = *((uint16_t*)data_ptr);
    //    ardprintf(_debug, "Arg[%d], len: %d:", argn++, len);
    crc = crc16_data(data_ptr, 2, crc);
    data_ptr += 2;
    while (len--) {
      crc = crc16_add(*data_ptr, crc);
      //      ardprintf(_debug, "%02X-", *data_ptr);
      data_ptr++;
    }
    //    INFO("");
    //    INFO("");
  }
  resp_crc = *(uint16_t*)data_ptr;
  //  ardprintf(_debug, "ARD Read CRC: %04X, calculated crc: %04X\r\n", resp_crc, crc);
  if (crc != resp_crc) {
    INFO((const char*)F("ARD: Invalid CRC"));
    return;
  }

  FP<void, void*> *fp;
  if (packet->callback != 0){
    fp = (FP<void, void*>*)packet->callback;

    return_cmd = packet->cmd;
    return_value = packet->_return;

    if (fp->attached())
      (*fp)((void*)packet);
  }
  else {
    if (packet->argc == 0) {
      is_return = true;
      return_cmd = packet->cmd;
      return_value = packet->_return;
    }
  }
}

uint32_t ESP::getTime() {
  uint16_t crc = request(CMD_GET_TIME, 0, 1, 0);
  request(crc);

  if (waitReturn() == false || return_cmd != CMD_GET_TIME || return_value == 0)
    return 0;
    
  return return_value;
}


void ESP::wifiConnect() {
  uint16_t crc;
  crc = request(CMD_WIFI_CONNECT, (uint32_t)&wifiCb, 0, 0);
  request(crc);
}

void ESP::write(uint8_t data) {
  switch (data) {
  case SLIP_START:
  case SLIP_END:
  case SLIP_REPL:
    _serial->write(SLIP_REPL);
    _serial->write(SLIP_ESC(data));
    break;
  default:
    _serial->write(data);
  }
}

void ESP::write(uint8_t* data, uint16_t len) {
  while (len--)
    write(*data++);
}

uint16_t ESP::request(uint16_t cmd, uint32_t callback, uint32_t _return, uint16_t argc) {
  uint16_t crc = 0;
  _serial->write(0x7E);
  write((uint8_t*)&cmd, 2);
  crc = crc16_data((uint8_t*)&cmd, 2, crc);

  write((uint8_t*)&callback, 4);
  crc = crc16_data((uint8_t*)&callback, 4, crc);

  write((uint8_t*)&_return, 4);
  crc = crc16_data((uint8_t*)&_return, 4, crc);

  write((uint8_t*)&argc, 2);
  crc = crc16_data((uint8_t*)&argc, 2, crc);
  return crc;
}

uint16_t ESP::request(uint16_t crc_in, uint8_t* data, uint16_t len) {
  uint8_t temp = 0;
  uint16_t pad_len = len;
  while (pad_len % 4 != 0)
    pad_len++;
  write((uint8_t*)&pad_len, 2);
  crc_in = crc16_data((uint8_t*)&pad_len, 2, crc_in);

  while (len--) {
    write(*data);
    crc_in = crc16_add(*data, crc_in);
    data++;
    if (pad_len > 0) pad_len--;
  }

  while (pad_len--) {
    write(temp);
    crc_in = crc16_add(*&temp, crc_in);
  }
  return crc_in;
}

uint16_t ESP::request(uint16_t crc_in, const __FlashStringHelper* data, uint16_t len) {
  uint8_t temp = 0;
  uint16_t pad_len = len;
  while (pad_len % 4 != 0)
    pad_len++;
  write((uint8_t*)&pad_len, 2);
  crc_in = crc16_data((uint8_t*)&pad_len, 2, crc_in);

  PGM_P p = reinterpret_cast<PGM_P>(data);
  while (len--) {
    uint8_t c = pgm_read_byte(p++);
    write(c);
    crc_in = crc16_add(c, crc_in);
    if (pad_len > 0) pad_len--;
  }

  while (pad_len--) {
    write(temp);
    crc_in = crc16_add(*&temp, crc_in);
  }
  return crc_in;
}

uint16_t ESP::request(uint16_t crc) {
  write((uint8_t*)&crc, 2);
  _serial->write(0x7F);
}

void ESP::init() {
  _proto.buf = _protoBuf;
  _proto.bufSize = sizeof(_protoBuf);
  _proto.dataLen = 0;
  _proto.isEsc = 0;
  if (_chip_pd != -1)
    pinMode(_chip_pd, OUTPUT);
}

ESP::ESP(Stream* serial, uint8_t chip_pd) :
_serial(serial), _chip_pd(chip_pd) {
  _debugEn = false;
  init();
}

ESP::ESP(Stream* serial, Stream* debug, uint8_t chip_pd) :
_serial(serial), _debug(debug), _chip_pd(chip_pd) {
  _debugEn = true;
  init();
}

void ESP::enable() {
  if (_chip_pd != -1)
    digitalWrite(_chip_pd, HIGH);
}

void ESP::disable() {
  if (_chip_pd != -1)
    digitalWrite(_chip_pd, LOW);
}

void ESP::INFO(const char* info) {
  if (_debugEn)
    _debug->println(info);
}

void ESP::reset() {
  uint16_t crc = request(CMD_RESET, 0, 0, 0);
  request(crc);
}

boolean ESP::ready() {
  uint32_t wait;

  for (uint8_t wait_time = 5; wait_time>0; wait_time--){
    is_return = false;
    return_value = 0;
    uint16_t crc = request(CMD_IS_READY, 0, 1, 0);
    request(crc);
    wait = millis();
    while (is_return == false && (millis() - wait < 1000)) {
      process();
    }
    if (is_return && return_value)
      return true;
  }
  return false;

}

boolean ESP::waitReturn(uint32_t timeout) {
  is_return = false;
  return_value = 0;
  return_cmd = 0;
  uint32_t wait = millis();
  while (is_return == false && (millis() - wait < timeout)) {
    process();
  }
  return is_return;
}

boolean ESP::waitReturn() {
  return waitReturn(ESP_TIMEOUT);
}

void ESP::process() {
  char value;
  while (_serial->available()) {
    value = _serial->read();
    switch (value) {
    case 0x7D:
      _proto.isEsc = 1;
      break;

    case 0x7E:
      _proto.dataLen = 0;
      _proto.isEsc = 0;
      _proto.isBegin = 1;
      break;

    case 0x7F:
      protoCompletedCb();
      _proto.isBegin = 0;
      break;

    default:
      if (_proto.isBegin == 0) {
        if (_debugEn) {
          _debug->write(value);
        }
        break;
      }
      if (_proto.isEsc) {
        value ^= 0x20;
        _proto.isEsc = 0;
      }

      if (_proto.dataLen < _proto.bufSize)
        _proto.buf[_proto.dataLen++] = value;

      break;
    }
  }
}
