import cv2
import numpy as np
import os, sys

THRESHOLD = 0.8
W = 10
H = 7

def check_ref_in_images(img, x, y, w, h, threshold):
    region = img[y:y+h, x:x+w]

    black_region = np.zeros((h, w), dtype=np.uint8)

    coincidences = cv2.compare(region, black_region, cv2.CMP_EQ)

    percentage = np.count_nonzero(coincidences) / coincidences.size
    print(percentage)
    
    return percentage >= threshold


if __name__ == '__main__':
    if len(sys.argv) < 2:
            print("Usando carpeta por defecto")
            folder = "prueba2"
    else:
        folder = sys.argv[1]
    i = 0

    for file in os.listdir(folder):
        if(file.endswith('.png')):
            img = cv2.imread(os.path.join(folder, file))

            # Define la región que deseas resaltar
            x = int(img.shape[1]/2) - 5 # coordenada x
            y = int(img.shape[0]-15) # coordenada y
            w = W # ancho del rectángulo
            h = H # altura del rectángulo
            threshold = THRESHOLD

            # cv2.rectangle(img, (x, y), (x + w, y + h), (0, 0, 255), 1)

            # # Muestra la imagen con el rectángulo rojo
            # cv2.imshow("Imagen con rectángulo rojo", img)
            # cv2.waitKey(0)
            # cv2.destroyAllWindows()

            img_gris = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            umbral, img_binaria = cv2.threshold(img_gris, 127, 255, cv2.THRESH_BINARY)

            result = check_ref_in_images(img=img_binaria, x=x, y=y, w=w, h=h, threshold=threshold)
            i=i+1
            print(result, i)

            if(result == False):
                print(f"La imagen {file} no tiene refuerzo")
                # print("Se va a eliminar la imagen")
                # os.remove(os.path.join(folder, file))
             


