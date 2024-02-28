import pyb
import sensor
import image
import time
from pyb import UART

sensor.reset()
sensor.set_pixformat(sensor.GRAYSCALE)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time=2000)

clock = time.clock()

areaDoMaiorRaio = 0
centroideDoMaiorRaioX = 0
centroideDoMaiorRaioY = 0
tempoMaiorRaio = 0

tempoInicial = time.time()

frames = 0
framesMaiorRaio = 0

uart = UART(1, 9600)                         # init with given baudrate
uart.init(9600, bits=8, parity=None, stop=1) # init with given parameters

while True:
    clock.tick()
    img = sensor.snapshot()

    thresholds = (200, 255)

    blobs = img.find_blobs([thresholds], area_threshold=225, merge=False)

    areaDoMaiorRaioFrame = 0
    centroideDoMaiorRaioFrameX = 0
    centroideDoMaiorRaioFrameY = 0
    tempoMaiorRaioFrame = 0

    for blob in blobs:
        img.draw_rectangle(blob.rect(), color=0)
        img.draw_circle(blob.cx(), blob.cy(), 1, color=0)
        areaRaioAtual = blob.pixels()

        raioIgualMaiorRaio = areaRaioAtual >= areaDoMaiorRaio - 100 and areaRaioAtual <= areaDoMaiorRaio + 100
        #print(raioIgualMaiorRaio)

        if areaRaioAtual > areaDoMaiorRaioFrame:
            #if(not(raioIgualMaiorRaio)):
            #    framesMaiorRaio = 0
            #    print("abcd")
            areaDoMaiorRaioFrame = areaRaioAtual
            centroideDoMaiorRaioFrameX = blob.cx()
            centroideDoMaiorRaioFrameY = blob.cy()

        #print(blob.cx(), blob.cy())


        #if(raioIgualMaiorRaio):
            #print("continua no raio", areaDoMaiorRaio)
            #framesMaiorRaio +=1

    if areaDoMaiorRaioFrame > areaDoMaiorRaio:
        areaDoMaiorRaio = areaDoMaiorRaioFrame
        centroideDoMaiorRaioX = centroideDoMaiorRaioFrameX
        centroideDoMaiorRaioY = centroideDoMaiorRaioFrameY


    if uart.any():
        dados_recebidos = uart.readline()
        print(dados_recebidos.decode('utf-8'))
        dados_split = dados_recebidos.decode('utf-8').strip().split(":")
        if len(dados_split) == 1:
            dados = dados_split[0]
            endereco = None
        elif len(dados_split) == 2:
            dados, endereco = dados_split
        else:
            continue
        if endereco == "5":
            uart.write("6:{},{},{}\n".format(areaDoMaiorRaioFrame, centroideDoMaiorRaioFrameX, centroideDoMaiorRaioFrameY))
            print(areaDoMaiorRaioFrame, centroideDoMaiorRaioFrameX, centroideDoMaiorRaioFrameY)

    #if uart.readchar() == "3":
     #   uart.write()
        #print()
    #else:
     #   uart.sendbreak()
    #uart.write("3:{},{},{}\n".format(areaDoMaiorRaioFrame, centroideDoMaiorRaioFrameX, centroideDoMaiorRaioFrameY))    print(areaDoMaiorRaioFrame, centroideDoMaiorRaioFrameX, centroideDoMaiorRaioFrameY)
    frames += 1
tempoFinal = time.time() - tempoInicial
print("Frames", frames)
print("Tempo final:", tempoFinal)

print(framesMaiorRaio, tempoFinal, frames)
print("Tempo maior raio:", (framesMaiorRaio*tempoFinal)/frames)

print("Area: ", areaDoMaiorRaio)
print("Centroide X: ", centroideDoMaiorRaioX)
print("Centroide Y: ", centroideDoMaiorRaioY)
