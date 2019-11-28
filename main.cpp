
//monitor_speed = 115200
 //escreve no topico
#include "mbedtls/aes.h"
#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>

#define SCL 22 //Clock

const int MPU_addr=0x68; // I2C address of the MPU-6050
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ; 
int xAng, yAng, zAng;
double x, y, z;
int minVal=265; int maxVal=402;
float g1,g2,g3;

const char* ssid = "lucas_rasp";
const char* password =  "entreaqui";
const char* mqttServer = "10.3.141.1";
const int mqttPort = 1883;
const char* mqttUser = "biosinal";
const char* mqttPassword = "notapass";

WiFiClient espClient;
PubSubClient client(espClient);

unsigned int contador = 1;
char mensagem[35];
char* aux;
char aux2[17];

int pa=32,pb=39,pc=34,pd=35;
unsigned int v1, v2, v3, v4;

char cG1[7], cG2[7], cG3[7];
void reconectabroker()
{
  //Conexao ao broker MQTT
  client.setServer(mqttServer, mqttPort);
  while (!client.connected())
  {
    Serial.println("Conectando ao broker MQTT...");
    if (client.connect("test_channel", mqttUser, mqttPassword )){
      Serial.println("Conectado ao broker!");
    }
    else
    {
      Serial.print("Falha na conexao ao broker - Estado: ");
      Serial.print(client.state());
      delay(2000);
    }
  }
}

float giroscopio(int pino){
  Wire.begin(pino, SCL);
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B); // PWR_MGMT_1 register
  Wire.write(0); // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
  
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B); // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr,14,true); // request a total of 14 registers
  
  AcX=Wire.read()<<8|Wire.read(); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
  AcY=Wire.read()<<8|Wire.read(); // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ=Wire.read()<<8|Wire.read(); // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  Tmp=Wire.read()<<8|Wire.read(); // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  GyX=Wire.read()<<8|Wire.read(); // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GyY=Wire.read()<<8|Wire.read(); // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyZ=Wire.read()<<8|Wire.read(); // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)

  xAng = map(AcX,minVal,maxVal,-90,90); 
  yAng = map(AcY,minVal,maxVal,-90,90); 
  zAng = map(AcZ,minVal,maxVal,-90,90);
  
  x = RAD_TO_DEG * (atan2(-yAng, -zAng)+PI); 
  y = RAD_TO_DEG * (atan2(-xAng, -zAng)+PI); 
  z = RAD_TO_DEG * (atan2(-yAng, -xAng)+PI);
  return y;
  //Serial.print("AcX = "); Serial.print(AcX);
  //Serial.print(" | AcY = "); Serial.print(AcY);
  //Serial.print(" | AcZ = "); Serial.print(AcZ);
  //Serial.print("AngX = "); Serial.print(x);
  
  //Serial.print(" | gamer = "); Serial.println(yAng);
  
}

void leitura(unsigned int* v1,unsigned int* v2,unsigned int* v3,unsigned int* v4){
  *v1 = analogRead(pa);
  *v2 = analogRead(pb);
  *v3 = analogRead(pc);
  *v4 = analogRead(pd);
}

void setup()
{
  Serial.begin(115200);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED)
  {
    delay(600);
    Serial.println("Iniciando conexao com a rede WiFi..");
  }

  Serial.println("Conectado na rede WiFi!");
}
char * TextoSinais (unsigned int a, unsigned int b, unsigned int c, unsigned int d ) {
  char ac[5];
  sprintf (ac, "%04i", a);
  char bc[5];
  sprintf (bc, "%04i", b);
  char cc[5];
  sprintf (cc, "%04i", c);
  char dc[5];
  sprintf (dc, "%04i", d);

  char *retorno = new char[17];
  sprintf(retorno,"%s%s%s%s",ac,bc,cc,dc);

  return retorno;
}

void loop()
{
  reconectabroker();
  leitura(&v1, &v2,&v3, &v4);
  aux = TextoSinais(v1,v2,v3,v4);

  g1 = giroscopio(23);
  g2 = giroscopio(19);
  g3 = giroscopio(18);

  //270 < g2 < 360
  if (g2 < 360 && g2 > 270)
    g2 -= 270;
  else if (g2 > 0 && g2 < 180)
    g2 += 90;
  else if (g2>180 && g2<360)
    g2 = 0;
  
  if (g1 < 360 && g1 > 270)
    g1 -= 270;
  else if (g1 > 0 && g1 < 180)
    g1 += 90;
  else if (g1>180 && g1<360)
    g1 = 0;

  if (g3 < 360 && g3 > 270)
    g3 -= 270;
  else if (g3 > 0 && g3 < 180)
    g3 += 90;
  else if (g3>180 && g3<360)
    g3 = 0;

  dtostrf(g1, 6, 2, cG1);
  dtostrf(g2, 6, 2, cG2);
  dtostrf(g3, 6, 2, cG3);
   
  // teste, retirar depois
  /*for (int i = 0; i < 17; i++) {
    aux2[i]=aux[i];  
  }
  //free(aux);
  Serial.println(aux2);*/
  Serial.println("-----");
  //aux = aes(aux2);
  for (int i = 0; i < 16 ; i++)mensagem[i] = aux[i];
  for (int i = 16; i < 22 ; i++)mensagem[i] = cG1[i-16];
  for (int i = 22; i < 28 ; i++)mensagem[i] = cG2[i-22];
  for (int i = 28; i < 34 ; i++)mensagem[i] = cG3[i-28];
  free(aux);
  //Envia a mensagem ao broker
  Serial.println(mensagem);
  client.publish("test_channel", mensagem);
  for (int i = 0; i < 32 ; i++)mensagem[i] = '0';
  //Aguarda 0.01 segundos para enviar uma nova mensagem
  delay(100);
}



char* aes(char* input) {

  mbedtls_aes_context aes;

  char * key = "abcdefgoijklmnopabcdefgoijklmnop";

  unsigned char output[16];
  char* output2 = new char[32];
  mbedtls_aes_init( &aes );
  mbedtls_aes_setkey_enc( &aes, (const unsigned char*) key, strlen(key) * 8 );
  mbedtls_aes_crypt_ecb(&aes, MBEDTLS_AES_ENCRYPT, (const unsigned char*)input, output);
  mbedtls_aes_free( &aes );

  for (int i = 0; i < 16; i++) {
    char str[2];
    sprintf(str, "%02x", (int)output[i]);
    output2[i * 2] = str[0];
    output2[i * 2 + 1] = str[1];

  }
  return output2;
}
char* sinalString(unsigned int cont) {
  String bla = String(cont);
  int sizee = bla.length();
  char *inp = new char[5];
 
  if (sizee < 4) {
    for (int i = 4-sizee; i < 4 ; i++)
    {
      inp[i] = bla[i+sizee-4];

    }

    for (int i = 0; i < 4-sizee; i++)    
    {
      
      inp[i] = '0';

    }

  } else {
    for (int i = 0; i < 4 ; i++)
    {

      inp[i] = bla[i];
    }

  }
  
  return inp;
}

