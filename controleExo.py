# -*- coding: utf-8 -*-
import threading
#import RPi.GPIO as GPIO
import numpy as np
import paho.mqtt.client as mqtt
from time import sleep
#from Tkinter import *
#import serial
import time
from tkinter import *
#import tkSnack


MQTT_SERVER ="10.3.141.1"
MQTT_PATH = "test_channel"

def on_connect(client, userdata, flags, rc):
    print("Connected myosinal "+str(rc))
    client.subscribe(MQTT_PATH)

def on_connect2(client, userdata, flags, rc):
    print("Connected motor "+str(rc))
    client.subscribe("motor")
 
def on_message(client, userdata, msg):
    global exc
    aux = str(msg.payload)
    giroscopio1 = aux[18:24]
    giroscopio2 = aux[25:31]

    print(msg.topic+" "+str(msg.payload))
    #exc.sinais(int(aux[1:4]),int(aux[5:8]),int(aux[9:12]),int(aux[13:16]), float())
    exc.atualiza(int(aux[2:6]),int(aux[6:10]),int(aux[10:14]),int(aux[14:18]))
    #exc._atualiza(float(giroscopio1),float(giroscopio2))

def on_publish(client,userdata,result):             #create function for callback
    print("data published \n")

class controle():
    
    def __init__(self,client2):
        self.mqtt_cliente = client2
        #constante da admitancia
        self.admitanciaConstante = 1 
        #constante da impedancia
        self.impedanciaConstante = 1
        #contante eletromiografia
        self.eletromiografiaConstante = 1

        #gpio raspberry
        '''
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(12, GPIO.OUT)
        GPIO.setup(33, GPIO.OUT)
        self.pwm1=GPIO.PWM(12, 50)
        self.pwm1.start(0)
        GPIO.output(12, False)
        self.pwm2=GPIO.PWM(33, 50)
        self.pwm2.start(0)
        GPIO.output(33, False)
        '''

        # Interface
        self.tela = Tk()
        self.widget1 = Frame(self.tela)
        self.tela.title = ("Lucas")
        self.widget1.pack()
        
        self.msg = Label(self.widget1, text="MyoControl")
        self.msg["font"] = ("Verdana", "20", "bold")
        self.msg.pack ()
        
        self.container = Frame(self.tela)
        self.container["padx"] = 60
        self.container["pady"] = 10
        self.container.pack()

        self.calibra = Button(self.container, text = "Normalizar", command = self.Normalizar)
        self.calibra["font"] = ("Calibri", "20")
        self.calibra["padx"] = 10
        self.calibra.pack (side=LEFT)
        

        self.executa = Button(self.container, text = "Executar", command = self.executa)
        self.executa["font"] = ("Calibri", "20")
        self.executa["padx"] = 10
        self.executa.pack (side=LEFT)

      
        self.calibraMotor = Button(self.container, text = "Calibrar Motor", command = self.calibraMotor)
        self.calibraMotor["font"] = ("Calibri", "20")
        self.calibraMotor["padx"] = 10
        self.calibraMotor.pack (side=LEFT)

        #tkSnack.initializeSnack(self.tela)
        #self.snd = tkSnack.Sound()
        #self.snd.read('fira.wav')

        #para as funçoes de controle 
        self.dutty1 = 0.0
        self.dutty2 = 0.0
        self.a1 = 0
        self.a2 = 0
        self.limite1 = 0
        self.limite2 = 0
        self.estatus = 0
        #sinais
        self.sinal1 = 0
        self.sinal2 = 0
        self.sinal3 = 0
        self.sinal4 = 0
        #Normalizaçao
        self.sn1 = 0
        self.sn2 = 0
        self.sn3 = 0
        self.sn4 = 0
        #limite sinal
        self.sinalLimite1 = 0
        self.sinalLimite2 = 0
        self.sinalLimite3 = 0
        self.sinalLimite4 = 0
        #feedback
        self.feedbackMotor1 = 0
        self.feedbackMotor2 = 0
        self.peso = 0
        #Angulos calculados
        self.calcMotor1 = 0
        self.calcMotor2 = 0
        #tempo
        self.time = 0
        self.musculoCalibrado = 0
        #Configura a serial e a velocidade de transmissao
        #Sself.ser = serial.Serial("/dev/serial0", 115200)
        #self.ser.flushInput()
        #verifica calibração
        self.calibrado = False

#    def inicializa(self):
#        GPIO.setmode(GPIO.BOARD)
#        GPIO.setup(3, GPIO.OUT)
#        GPIO.setup(5, GPIO.OUT)
#        self.pwm1=GPIO.PWM(3, 50)
#        self.pwm1.start(0)
#        GPIO.output(3, True)
#        self.pwm2=GPIO.PWM(5, 50)
#        self.pwm2.start(0)
#        GPIO.output(5, True)
        
    def mythread(self):
        while(self.threadisalive):
            pass
            #self.pwm1.ChangeDutyCycle( self.dutty1 )
            #self.pwm2.ChangeDutyCycle( self.dutty2 )
            
    def run(self):
        
        if ( not self.threadisalive):
            self.threadisalive = True
            try:
                self.t1 = threading.Thread(target = self.mythread)
                self.t1.start()
            except:
                print("erro threads")
    def kill_run(self):
        self.threadisalive= False
    def Normalizar(self):
        
        
        print("normalizar")
        self.calibrado = True
        self.estatus = 2
        self.time = time.time() +3;

    def calculo_controle(self, atual, s1, s2, a):
        return  float (atual + ((s1*self.eletromiografiaConstante- s2*self.eletromiografiaConstante  - (self.admitanciaConstante *np.sin(np.deg2rad(a+atual))) )/1000))
        #return  float (np.sqrt(s1) + (np.sqrt(s1) * np.sin(atual+a)))/2
    
    def impedancia(self, atual, proximo):
        return float ( atual + (self.impedanciaConstante * np.sin(np.deg2rad(proximo))* self.peso)/1000  )
        pass

    def atualiza(self, a, b, c, d):
        print(a)
        print(self.sinalLimite1)
        print(b)
        print(self.sinalLimite2)
        
        if(self.estatus==1):
            #print("calc")
            #self.mandamensagem()
            
            if(a>self.sinalLimite1 or b>self.sinalLimite2 or c>self.sinalLimite3 or d<self.sinalLimite4):
                #self.snd.play(blocking=1)
                self.estatus = 0
                self.dutty1 = (180)/ 18 + 2 
                self.dutty2 = (180)/ 18 + 2
            
                #self.pwm1.ChangeDutyCycle( self.dutty1 )
                #self.pwm2.ChangeDutyCycle( self.dutty2 )
                
            self.sinal1 = a * self.sn1
            self.sinal2 = b * self.sn2
            self.sinal3 = c * self.sn3
            self.sinal4 = d * self.sn4
            
            

            self.a1 = self.calculo_controle(self.a1, self.sinal1,self.sinal2,0)
            self.a2 = self.calculo_controle(self.a2, self.sinal3,self.sinal4,self.a1)
            
            self.a2 = self.impedancia(self.a2,0)
            self.a1 = self.impedancia(self.a1, self.a2)
            
            self.a1 = self.seguranca(self.a1)
            self.a2 = self.seguranca(self.a2)
            
            self.calcMotor1 = self.a1
            self.calcMotor2 = self.a2
            
            self.publica()
            
            self.dutty1 = (180 - self.a1)/ 18 + 2 
            self.dutty2 = (180 - self.a2)/ 18 + 2
            
            #self.pwm1.ChangeDutyCycle( self.dutty1 )
            #self.pwm2.ChangeDutyCycle( self.dutty2 )
        elif(self.estatus == 2):
            self.calibrado = True;
            if (time.time() < self.time):
                
                if(self.musculoCalibrado==0):
                    if(self.sinalLimite1<a):
                        self.sinalLimite1 = a  
                if(self.musculoCalibrado==1):
                    if(self.sinalLimite2<b):
                        self.sinalLimite2 = b
                if(self.musculoCalibrado==2):
                    if(self.sinalLimite3<c):
                        self.sinalLimite3 = c
                if(self.musculoCalibrado==3):
                    if(self.sinalLimite4<d):
                        self.sinalLimite4 = d
            elif(self.musculoCalibrado<3):
                print("proximo")
                self.musculoCalibrado+=1
                self.time = time.time() + 3
            else:
                
                self.sn1 = 100/self.sinalLimite1
                self.sn2 = 100/self.sinalLimite2
                self.sn3 = 100/self.sinalLimite3
                self.sn4 = 100/self.sinalLimite4
                self.estatus = 0
                self.musculoCalibrado=0
                textoPopup = ("Sensor 1 = "+ str(self.sinalLimite1) +       "Sensor 2 = "+ str(self.sinalLimite2) +   "Sensor 3 = "+ str(self.sinalLimite3) + "Sensor 4 = "+ str(self.sinalLimite4))
                print(textoPopup)
                #self.mensagem.configue(text = textoPopup)
                self.mensagem.pack()
                self.popupmsg(textoPopup)
                
                pass
        else:
                self.estatus = 0
                
                
        pass
        
    
    def popupmsg(self, msg):
        w = Toplevel()
        w.wm_title("Parsed Code")
        w.geometry('400x300')

        msg = Message(w, text=msg)
        msg.pack()

        button = Button(w, text="Dismiss", command=w.destroy)
        button.pack()
        
    
    def _atualiza(self,a1,a2):
        if(self.estatus==1):
            self.a1 = a1
            self.a2 = a2
            self.ajusta()
            
            self.a1 = self.seguranca(self.a1)
            self.a2 = self.seguranca(self.a2)
            
            self.dutty1 = self.a1/ 18+ 2
            self.dutty2 = self.a2/ 18 + 2
            #self.pwm1.ChangeDutyCycle( self.dutty1 )
            #self.pwm2.ChangeDutyCycle( self.dutty2 )
        elif(self.estatus == 2):
            pass

    def seguranca(self, a):
        if(a<0):
            return 0
        elif(a>180):
            return 180
        return a
        
    def ajuste (self):
        pass
    
    def ajusta(self):# para o giroscopio
        self.a1 = 180- self.a1
        self.a2 = 180- self.a2
        self.a2 = self.a2 - (self.a1-180 )
        
    def clean(self):
        #GPIO.cleanup()
        pass
    
    def calibraMotor(self):
        self.dutty1 = 0/ 18 + 2 
        self.dutty2 = 0/ 18 + 2 
        #Envia 1 pela serial
        '''
        self.ser.write(str.encode("0"))
        self.pwm1.ChangeDutyCycle( self.dutty1 )
        self.pwm2.ChangeDutyCycle( self.dutty2 )
        sleep(2)
        self.dutty1 = 180/ 18 + 2 
        self.dutty2 = 180/ 18 + 2 
        sleep(2)
        self.pwm1.ChangeDutyCycle( self.dutty1 )
        self.pwm2.ChangeDutyCycle( self.dutty2 )
        '''
    def executa(self):
        #self.publica()
        if(self.estatus == 1) :
            self.estatus = 0
        elif(self.calibrado):
            self.estatus = 1
            print("executando")

    def calibra(self,m1,m2,m3,m4):
        self.limite1 = m1
        self.limite2 = m2
        pass
    '''
    def mandamensagem(self):
        #Envia 0 pela serial
        self.ser.flushInput()
        self.ser.write(str.encode("1"))
        #Aguarda reposta
        self.feedbackMotor1 = float(self.ser.readline())
        print(self.feedbackMotor1)
        self.feedbackMotor2 = float(self.ser.readline())
        print(self.feedbackMotor2 )
        self.ser.write(str.encode("2"))
        self.peso = float(self.ser.readline())
        print(self.peso)
        #self.publica()
        # '''
        #pass 

    def publica(self):
        self.mqtt_cliente.publish("motor",format(self.feedbackMotor1,'06.2f') +format(self.feedbackMotor2 ,'06.2f')+ format(self.peso,'08.2f') + format(self.calcMotor1,'06.2f')+ format(self.calcMotor2,'06.2f'))
        
        
        
    def movePelaInterface(self):
        try:
            a = int(self.campo1.get())
            b = int(self.campo2.get())
            self.dutty1 = a/ 18+ 2
            self.dutty2 = b/ 18+ 2
            self.pwm1.ChangeDutyCycle( self.dutty1 )
            self.pwm2.ChangeDutyCycle( self.dutty2 )
        except:
            print("erro")
            pass
            
#a.kill()
try:   
    client = mqtt.Client()
    client.on_connect = on_connect
    client.on_message = on_message
    client2 = mqtt.Client()
    client2.on_connect = on_connect2
    client2.on_publish = on_publish  
    client.username_pw_set("bio2","notanewpass")
    #client.connect(MQTT_SERVER, 1883, 60)
    client2.username_pw_set("bio2","notanewpass")
    #client2.connect(MQTT_SERVER, 1883, 60)
    
    #client.loop_start()
    #client2.loop_start()

    exc = controle(client2)

    #exc.mandamensagem()
    exc.tela.mainloop()
    
finally:
    pass
    #GPIO.cleanup() 





