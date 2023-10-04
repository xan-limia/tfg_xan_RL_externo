import os
import cv2
import numpy as np

def mask_images(img):
        h, w = img.shape[:2]
        maskImg = np.zeros_like(img)
        maskImg[int(h / 3):] = img[int(h / 3):]
        return maskImg

def calcular_media_distancia_euclidiana(folder):
    stored_images = []
    # Obtener la lista de archivos en el directorio
    for filename in os.listdir(folder):
            if(filename.endswith('.png')):
                img = cv2.imread(os.path.join(folder, filename))
                # mask_img = mask_images(img)
                # stored_images.append(mask_img)
                stored_images.append(img)
    num_imagenes = len(stored_images)

    # Verificar que haya al menos dos imágenes en el directorio
    if num_imagenes < 2:
        print("Debe haber al menos dos imágenes en el directorio.")
        return

    # Variables para almacenar la suma de las distancias y el número de comparaciones
    suma_distancias = 0
    num_comparaciones = 0

    # Iterar sobre todas las combinaciones posibles de imágenes
    for i in range(num_imagenes - 1):
        for j in range(i + 1, num_imagenes):
            # Cargar las imágenes utilizando OpenCV
            imagen1 = stored_images[i]
            imagen2 = stored_images[j]

            # Verificar que las imágenes se hayan cargado correctamente
            if imagen1 is None or imagen2 is None:
                print("Error al cargar las imágenes.")
                return

            # Calcular la distancia euclidiana utilizando la función proporcionada
            distancia = np.sum((cv2.absdiff(imagen1.flatten(), imagen2.flatten()) ** 2))

            # Agregar la distancia a la suma total
            suma_distancias += distancia
            num_comparaciones += 1

    # Calcular la media de las distancias
    media_distancias = suma_distancias / num_comparaciones

    return media_distancias

# Directorio que contiene las imágenes
directorio_imagenes = "prueba_siguelinea_2"

# Llamar a la función para calcular la media de la distancia euclidiana
media_distancia = calcular_media_distancia_euclidiana(directorio_imagenes)

# Imprimir el resultado
print("La media de la distancia euclidiana entre todas las imágenes es:", media_distancia)
