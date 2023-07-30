import os, cv2, pyexiv2, shutil

def leer_archivos_directorio_orig(directorio_orig):
    archivos = os.listdir(directorio_orig)
    return archivos

def obtener_datos_archivo(directorio_orig, archivo):
    img_exif = pyexiv2.Image(os.path.join(directorio_orig, archivo))
    metadata = img_exif.read_exif()
    text = metadata['Exif.Photo.UserComment']               
    linear_vel = float(text.split("=")[1].split("\n")[0])
    angular_vel = float(text.split("=")[2])

    return angular_vel

def crear_carpeta(directorio_dest, dato):
    os.makedirs(directorio_dest, exist_ok=True)
    nueva_carpeta = os.path.join(directorio_dest, dato)
    os.makedirs(nueva_carpeta, exist_ok=True)
    return nueva_carpeta

def copiar_archivo(directorio_orig, archivo, carpeta_destino):
    #ruta_archivo = os.path.abspath(archivo)
    shutil.copy(os.path.join(directorio_orig, archivo), carpeta_destino)

def organizar_archivos(directorio_orig, directorio_dest):
    archivos = leer_archivos_directorio_orig(directorio_orig)
    for archivo in archivos:
        if(archivo.endswith('.png')):
            dato = obtener_datos_archivo(directorio_orig, archivo)
            carpeta_destino = crear_carpeta(directorio_dest, f"{dato}")
            copiar_archivo(directorio_orig, archivo, carpeta_destino)

directorio_orig = 'prueba_siguelinea_manual_3'
directorio_dest = directorio_orig + '_separadas'

organizar_archivos(directorio_orig, directorio_dest)

    
        