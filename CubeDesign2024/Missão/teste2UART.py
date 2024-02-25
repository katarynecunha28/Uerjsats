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

count = 0

uart = UART(1, 9600)                         # init with given baudrate
uart.init(9600, bits=8, parity=None, stop=1) # init with given parameters

while count < 1000:
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
        if areaRaioAtual > areaDoMaiorRaioFrame:
            areaDoMaiorRaioFrame = areaRaioAtual
            centroideDoMaiorRaioFrameX = blob.cx()
            centroideDoMaiorRaioFrameY = blob.cy()

        #print(blob.cx(), blob.cy())

    if areaDoMaiorRaioFrame > areaDoMaiorRaio:
        areaDoMaiorRaio = areaDoMaiorRaioFrame
        centroideDoMaiorRaioX = centroideDoMaiorRaioFrameX
        centroideDoMaiorRaioY = centroideDoMaiorRaioFrameY

    if uart.any():
        dados_recebidos = uart.readline()

        endereco, dados = dados_recebidos.decode('utf-8').strip().split(":")
        endereco = int(endereco)

        if endereco == 3:
            uart.write("2:{},{},{}\n".format(areaDoMaiorRaioFrame, centroideDoMaiorRaioFrameX, centroideDoMaiorRaioFrameY))    print(areaDoMaiorRaioFrame, centroideDoMaiorRaioFrameX, centroideDoMaiorRaioFrameY)

    #uart.write("2:{},{},{}\n".format(areaDoMaiorRaioFrame, centroideDoMaiorRaioFrameX, centroideDoMaiorRaioFrameY))    print(areaDoMaiorRaioFrame, centroideDoMaiorRaioFrameX, centroideDoMaiorRaioFrameY)
    count += 1

print("Area: ", areaDoMaiorRaio)
print("Centroide X: ", centroideDoMaiorRaioX)
print("Centroide Y: ", centroideDoMaiorRaioY)
