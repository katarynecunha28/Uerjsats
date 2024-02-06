import cv2
import numpy as np
import time

def calcular_centroide(contorno):
    M = cv2.moments(contorno)
    if M["m00"] != 0:
        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])
        return cX, cY
    else:
        return None

def detectar_raios(gif_path):
    # Lê o GIF usando OpenCV
    cap = cv2.VideoCapture(gif_path)

    # Inicializa o tempo para o raio com a maior área
    tempo_raio_maior_area = 0

    # Loop sobre os frames
    while True:
        inicio_frame = time.time()  # Marca o início do processamento do frame
        ret, frame = cap.read()
        if not ret:
            break

        # Converte o frame para escala de cinza
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Aplica o detector de bordas (por exemplo, Canny)
        edges = cv2.Canny(gray, 50, 150)

        # Encontrar contornos
        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Encontrar o contorno de maior área
        maior_contorno = max(contours, key=cv2.contourArea)

        # Desenhar contornos nos raios detectados
        cv2.drawContours(frame, [maior_contorno], -1, (0, 255, 0), 2)

        # Calcular o centroide do contorno de maior área
        centroide = calcular_centroide(maior_contorno)

        # Desenhar um círculo no centroide
        if centroide is not None:
            cv2.circle(frame, centroide, 5, (0, 0, 255), -1)

            # Adicionar texto com as coordenadas do centroide na imagem
            texto = f"Centroide: {centroide[0]}, {centroide[1]}"
            cv2.putText(frame, texto, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2, cv2.LINE_AA)

        fim_frame = time.time()  # Marca o fim do processamento do frame
        tempo_frame = fim_frame - inicio_frame

        # Atualiza o tempo do raio com a maior área
        if tempo_frame > tempo_raio_maior_area:
            tempo_raio_maior_area = tempo_frame

        # Adiciona o tempo do raio com a maior área como texto na imagem
        texto_tempo = f"Tempo do Raio com Maior Area: {tempo_raio_maior_area:.2f} segundos"
        cv2.putText(frame, texto_tempo, (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)

        # Mostrar a imagem com os contornos
        cv2.imshow('Detecção de Raios', frame)

        # Pressione 'q' para sair do loop
        if cv2.waitKey(30) & 0xFF == ord('q'):
            break

    # Libera os recursos
    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    gif_path = r"C:\Users\katar\OneDrive\Documentos\UERJ\UERJ SATS\cubedesign\5XK0pnt.gif"  # Localização do gif
    detectar_raios(gif_path)
