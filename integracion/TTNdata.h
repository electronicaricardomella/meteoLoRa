// Datos de conexión a TTN para el primer nodo PCB

// This EUI must be in little-endian format, so least-significant-byte
// first. When copying an EUI from ttnctl output, this means to reverse
// the bytes. For TTN issued EUIs the last bytes should be 0xD5, 0xB3,0x70.
static const u1_t PROGMEM APPEUI[8]={ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
void os_getArtEui (u1_t* buf) { memcpy_P(buf, APPEUI, 8);}
// This should also be in little endian format, see above.
static const u1_t PROGMEM DEVEUI[8]={ 0x7E, 0x1E, 0x06, 0xD0, 0x7E, 0xD5, 0xB3, 0x70};
void os_getDevEui (u1_t* buf) { memcpy_P(buf, DEVEUI, 8);}
// This key should be in big endian format (or, since it is not really a
// number but a block of memory, endianness does not really apply). In
// practice, a key taken from ttnctl can be copied as-is.
static const u1_t PROGMEM APPKEY[16] = { 0xD2, 0xA4, 0x34, 0x97, 0x37, 0x72, 0x28, 0xE3, 0xA7, 0x27, 0x41, 0xE1, 0xA5, 0x8F, 0x45, 0x1C};
void os_getDevKey (u1_t* buf) {  memcpy_P(buf, APPKEY, 16);}
