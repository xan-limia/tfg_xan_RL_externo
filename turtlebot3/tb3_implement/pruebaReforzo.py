import cv2
import numpy as np
import os, sys

THRESHOLD = 0.04
W = 80
H = 12

def check_ref_in_images(img, x, y, w, h, threshold):
    region = img[y:y+h, x:x+w]

    # black_region = np.zeros((h, w), dtype=np.uint8)
    # coincidences = cv2.compare(region, black_region, cv2.CMP_EQ)

    white_region = np.ones((h, w), dtype=np.uint8) * 255
    coincidences = cv2.compare(region, white_region, cv2.CMP_EQ)

    percentage = np.count_nonzero(coincidences) / coincidences.size
    print(percentage)
    print("coincidences: ", np.count_nonzero(coincidences))
    
    return percentage >= threshold


if __name__ == '__main__':
    if len(sys.argv) < 2:
            print("Usando carpeta por defecto")
            folder = "prueba_accion_manual_siguelinea2_m3_d150_r005"
    else:
        folder = sys.argv[1]
    i = 0

    for file in os.listdir(folder):
        if(file.endswith('.png')):
            img = cv2.imread(os.path.join(folder, file))

            # Define la región que deseas resaltar
            # x = int(img.shape[1]/2) - 4 # coordenada x
            # y = int(img.shape[0]-8) # coordenada y
            # print(x, y)
            x = 0
            y = 47
            
            print(img.shape)

            threshold = THRESHOLD
            h1, w1 = img.shape[:2]
            maskImg = np.zeros_like(img)
            # maskImg[int(h1 / 3):, int(w1/5):w1 - int(w1/5)] = img[int(h1 / 3):, int(w1/5):w1 - int(w1/5)]
            maskImg[int(h1 / 3):] = img[int(h1 / 3):]

            cv2.rectangle(maskImg, (x, y), (x + W, y + H), (0, 0, 255), 1)
            imgr = cv2.resize(maskImg, (320, 240))

            # Muestra la imagen con el rectángulo rojo
            cv2.imshow("Imagen con rectángulo rojo", imgr)
            cv2.waitKey(0)
            cv2.destroyAllWindows()

            img_gris = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            umbral, img_binaria = cv2.threshold(img_gris, 127, 255, cv2.THRESH_BINARY)

            # cv2.imshow("img binaria", img_binaria)

            result = check_ref_in_images(img=img_binaria, x=x, y=y, w=W, h=H, threshold=threshold)
            i=i+1
            print(result, i)

            if(result == False):
                print(f"La imagen {file} tiene refuerzo")
                # print("Se va a eliminar la imagen")
                # os.remove(os.path.join(folder, file))
             


