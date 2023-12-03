
#include <Wire.h>

/*************************************************************************************************************
*******************************CLASSE EE24CXXX - CI I2C EEPROM AT24C128/256***********************************
**************************************************************************************************************/
class EE24CXXX {
  private:
    byte _device_address;
  public:
    EE24CXXX(byte device_address){ _device_address = device_address; }
    void write(unsigned int eeaddress, unsigned char * data, unsigned int data_len);
    void read (unsigned int eeaddress, unsigned char * data, unsigned int data_len);
    
    template <class T> int write(unsigned int eeaddress, const T& value);
    template <class T> int read(unsigned int eeaddress, T& value);
};

void EE24CXXX::write(unsigned int eeaddress, unsigned char * data, unsigned int data_len) {
  unsigned int  address;
  unsigned int  page_space;
  unsigned int  page=0;
  unsigned int  num_writes;
  unsigned char first_write_size;
  unsigned char last_write_size;  
  unsigned char write_size;  
  
  page_space = int(((eeaddress/64) + 1)*64)-eeaddress;      // Calculate space available in first page

  if (page_space>16) {                                      // Calculate first write size
     first_write_size = page_space-((page_space/16)*16);
     if (first_write_size==0) { first_write_size=16; }
  } else {
    first_write_size = page_space; 
  }
  if (data_len>first_write_size)  { last_write_size = (data_len-first_write_size)%16;   }      // calculate size of last write  
  num_writes = (data_len>first_write_size) ? ((data_len-first_write_size)/16)+2 : 1;           // Calculate how many writes we need
     
  unsigned char i=0, counter=0;
  address = eeaddress;
  for (page=0; page<num_writes; page++)  {
    if (page == 0) { 
      write_size = first_write_size; 
    } else if(page == (num_writes-1)) { 
      write_size = last_write_size;
    } else { 
      write_size = 16;
    }
 
    Wire.beginTransmission(_device_address);
    Wire.write((int)((address) >> 8));   // MSB
    Wire.write((int)((address) & 0xFF)); // LSB
    counter = 0;
    do { 
      Wire.write((byte) data[i++]);
      counter++;
    } while(counter<write_size);
    Wire.endTransmission();
    address += write_size;   // Increment address for next write
     
    delay(5);  // needs 5ms for page write
  }
}

void EE24CXXX::read(unsigned int eeaddress, unsigned char * data, unsigned int data_len){
  unsigned char i = 0;
  unsigned int size = data_len;
  unsigned int j=0;
  while (size > 0){
    Wire.beginTransmission(_device_address);
    eeaddress += j*28;
    
    Wire.write((int)(eeaddress >> 8));   // MSB
    Wire.write((int)(eeaddress & 0xFF)); // LSB
    Wire.endTransmission();
    
    if (size >= 28) { 
      Wire.requestFrom(_device_address, (unsigned int) 28);
      size -= 28;
    } else {
      Wire.requestFrom(_device_address, (unsigned int) size);
      size = 0;
    }
    while(Wire.available()) { data[i++] = Wire.read(); }
    j++;
  }
}

template <class T> int EE24CXXX::write(unsigned int eeaddress, const T& value) {
  const byte* p = (const byte*)(const void*)&value;
  unsigned char data[sizeof(value)+1];
  for (int i=0; i<sizeof(value); i++) { data[i] = *p++; }
  data[sizeof(value)] = '\n';
  write(eeaddress, data, sizeof(value));
  return sizeof(value);
}

template <class T> int EE24CXXX::read(unsigned int eeaddress, T& value) {
  byte * p = (byte*)(void*)&value;
  unsigned char c[sizeof(value)];
  read(eeaddress, c, sizeof(value));
  for (int i=0; i<sizeof(value); i++) { *p++ = c[i]; }
  return sizeof(value);
}
/*************************************************************************************************************
*******************************FIM - CLASSE EE24CXXX - CI I2C EEPROM AT24C128/256*****************************
**************************************************************************************************************/

EE24CXXX m(0x50);

struct teste{
  float a;
  float b;
  float c;
};

int i = 0;

void setup() {
  Serial.begin(9600);
  Wire.begin();  
}

void loop() {
  // put your main code here, to run repeatedly:
  teste j;
  int count = m.read(12*i, j);   //carrega da memória a quantidade de bytes 
  
  //Serial.print("Quant: ");
  //Serial.println(count);
  Serial.println();
  Serial.print("Valor: ");
  Serial.println(j.a);
  Serial.println(j.b);
  Serial.println(j.c);
  i++;
  while(1 && i==10){}
}
