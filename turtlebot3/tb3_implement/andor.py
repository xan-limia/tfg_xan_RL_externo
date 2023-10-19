import cv2, os
import numpy as np

W = 8
H = 6

# Función para cargar las imágenes de la lista
def cargar_imagenes(directorio, tipo_imagen):
        stored_images = []
        for filename in os.listdir(directorio):
            if filename.endswith('.png') and filename.startswith('or_') and not filename.endswith('robot_real.png'):
                img = cv2.imread(os.path.join(directorio, filename))
                #umbral, img = cv2.threshold(img, 127, 255, cv2.THRESH_BINARY)
                stored_images.append(img) # Gardar imaxe sen mascara
        return stored_images


# Función para calcular el "and" de las imágenes
def calcular_and_or(imagenes, flag):
 
    if flag == 0:
        resultado_or = imagenes[0]
        for imagen in imagenes:
            resultado_or = cv2.bitwise_or(imagen, resultado_or)

        return resultado_or
    
    elif flag == 1:
        resultado_and = imagenes[0]

        for imagen in imagenes:
            resultado_and = cv2.bitwise_and(imagen, resultado_and)

        return resultado_and

    
# directorio = 'result_analize_images'
# directorio = 'tb3_implement/manual_teleop_y2'
directorio = 'andor'
# directorio = 'prueba_manual_1'

imagenes = cargar_imagenes(directorio, '')

# flag = 0, calcula or, flag = 1, calcula and
resultado = calcular_and_or(imagenes, flag = 0)

imagen_redimensionada = cv2.resize(resultado, (320, 240))

x = 36
y = 52
w = W 
h = H

#cv2.rectangle(resultado, (x, y), (x + w, y + h), (0, 0, 255), 1)

img_gris = cv2.cvtColor(imagen_redimensionada, cv2.COLOR_BGR2GRAY)
umbral, img_binaria = cv2.threshold(img_gris, 127, 255, cv2.THRESH_BINARY)
cv2.imshow("Black&White", img_binaria)


# print(resultado)

cv2.imshow("OR", imagen_redimensionada)
cv2.waitKey(0)
cv2.destroyAllWindows()

nome_imaxe_or = 'and_or_y.png'
# directorio_resultado = 'prueba_siguelinea_manual_3_separadas'
# directorio_resultado = 'andor'
# cv2.imwrite(os.path.join(directorio_resultado, nome_imaxe_or), resultado)

