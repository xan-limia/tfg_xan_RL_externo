Execución do entorno en Windows:

	1. Iniciar docker, 
	2. Iniciar Xming. Importante activar a opción de "No acces control". Resto vai por defecto
	3. No PowerShell de Windows -> doker pull xanlimia/turtlebot3_base (en caso de non ter descargada a ultima version da imaxe)
	4. Entrar en wsl
	5. Executar ./xan_1.sh para lanzar o contenedor docker

Execución do codigo:
	1. É preciso utilizar varios terminais. Podese utilizar tmux para abrir o multiplexador de terminales ou lanzar varios contenerdores docker.
	2. cd src/tb3_implements
	3. Nunha terminal lanzar -> roslaunch turtlebot3_autorace.launch
	4. Noutra terminal -> pip install pyexiv2 e a continuación python3 reforzoAleatorio.py <nome_da_carpeta>
	5. Para ver a camara lanzar noutra terminal -> rqt


