
import sensor
import image
import time
import math
import pyb

# Suponha que a escala espacial seja 1 pixel = 1 metro para este exemplo
# Você deve ajustar isso com base nas informações específicas do seu ambiente.
escala_espacial = 1.0

# Localização inicial (latitude, longitude) como ponto de referência
referencia_lat = 41.8379
referencia_long = -87.6828

def calcular_centroide(blob):
    if blob.pixels() != 0:
        cX = blob.cx()
        cY = blob.cy()
        return cX, cY
    else:
        return None

def pixels_para_lat_long(coordenadas_pixels, escala_espacial, referencia_lat, referencia_long):
    pixel_x, pixel_y = coordenadas_pixels
    metros_x = pixel_x * escala_espacial
    metros_y = pixel_y * escala_espacial

    lat = referencia_lat + (metros_y / (111111.0))  # Assumindo que 1 grau de latitude é aproximadamente 111111 metros
    long = referencia_long + (metros_x / (111111.0 * math.cos(math.radians(referencia_lat))))

    return lat, long

sensor.reset()
sensor.set_pixformat(sensor.GRAYSCALE)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time=2000)
sensor.set_auto_gain(False)
sensor.set_auto_whitebal(False)

# Inicializa o tempo para o raio com a maior área
tempo_raio_maior_area = 0
centróides_lat_long = []

# Obtenha as dimensões originais do frame
frame = sensor.snapshot()
height = frame.height()
width = frame.width()

# Defina o fator de escala para aumentar a dimensão do plot (por exemplo, 1.5)
fator_escala = 1.5
nova_largura = int(width * fator_escala)
nova_altura = int(height * fator_escala)

while True:
    inicio_frame = pyb.millis()  # Marca o início do processamento do frame
    frame = sensor.snapshot()

    # Redimensiona o frame
    frame = frame.copy((nova_largura, nova_altura))

    # Aplica o detector de bordas (por exemplo, Canny)
    edges = frame.find_edges(image.EDGE_CANNY)

    # Encontrar círculos
    circles = frame.find_circles(threshold=2000, x_margin=10, y_margin=10, r_margin=10, r_min=2, r_max=50, r_step=2)

    if circles:
        maior_circulo = max(circles, key=lambda c: c.magnitude())

        # Desenhar círculos nos raios detectados
        frame.draw_circle(maior_circulo.x(), maior_circulo.y(), maior_circulo.magnitude(), color=(0, 255, 0))

        # Calcular o centroide do círculo de maior área
        centroide = (maior_circulo.x(), maior_circulo.y())

        # Converta as coordenadas de pixel para latitude e longitude
        if centroide is not None:
            centroide_lat_long = pixels_para_lat_long(centroide, escala_espacial, referencia_lat, referencia_long)

            # Agora, você pode usar as coordenadas em termos de latitude e longitude como necessário.
            print("Centroide (lat, long):", centroide_lat_long)

            # Desenhar um círculo no centroide
            frame.draw_circle(centroide[0], centroide[1], 5, color=(0, 0, 255), fill=True)

            # Adicionar texto com as coordenadas do centroide na imagem
            texto = f"Centroide: {centroide[0]}, {centroide[1]} (lat, long): {centroide_lat_long[0]}, {centroide_lat_long[1]}"
            frame.draw_string(10, 30, texto, color=(0, 0, 255), scale=fator_escala)

    fim_frame = pyb.millis()  # Marca o fim do processamento do frame
    tempo_frame = fim_frame - inicio_frame

    # Atualiza o tempo do raio com a maior área
    if tempo_frame > tempo_raio_maior_area:
        tempo_raio_maior_area = tempo_frame

    # Adiciona o tempo do raio com a maior área como texto na imagem
    texto_tempo = f"Tempo do Raio com Maior Area: {tempo_raio_maior_area:.2f} segundos"
    frame.draw_string(10, 60, texto_tempo, color=(0, 255, 0), scale=fator_escala)

# Enviar a imagem via USB para visualização no software OpenMV IDE
    frame.compress(quality=35)
    print("Enviando frame...")
    sensor.snapshot().save("frame.jpg", quality=35)
    pyb.delay(100)  # Aguarde um curto período para garantir que a imagem seja exibida completamente

#Liberar memória
    frame = None
       
