# tfg_xan_RL_externo

Traballo Fin de Grao 2023 

Uso de coñecemento externo na aprendizaxe por reforzo dun comportamento visual en robótica móvil 

Xan Limia García


**Guia de usuario**
    
    A continuación detállase como poñer en funcionamento o noso sistema.

    Os requisitos iniciais son ter descargada a imaxe Docker e clonar o repositorio principal.


    doker pull xanlimia/turtlebot3\_simulation
    git clone git@github.com:xan-limia/tfg\_xan\_RL\_externo.git


Execución do contorno de simulación

    Windows
    
        Será preciso ter instalado \acrfull{wsl} cunha imaxe de Ubuntu 20.04

         Inciar Docker Desktop.
         Iniciar Xming. Importante activar a opción _No acces control_. Resto vai por defecto.
         Dende _wsl_ executar o ficheiro _xan\_1.sh_

    Linux (Ubuntu 20.04)

        Inciar docker.
        Executar: \textit{xhost +}.
        Executar o ficheiro \textit{xan\_1.sh}.



Execución do código de aprendizaxe

    Executar: _catkin_make_ 
    Lanzar os ficheiros launch situados na carpeta _/src/tb3\_implement/launch/_
    Algoritmo de acción supervisada: _python3 reforzoAccionManual.py <folder\_name>_
    Algoritmo de qlearning: _python3 qlearning.py <folder\_name>_
    Creación de estados iniciais con off policy: _python3 off\_policy.py <bag\_file.bag>_
    Funcións de análise no ficheiro _analisis.py_


Conexión co Turtlebot3 real

    Só dispoñible para a versión 20.04 de ubuntu. Será preciso ter configurado o robot seguindo os pasos da guía de inicio rápido \cite{turtlebot_manual}.

    Executar en PC remoto: _roscore_
    En Turtlebot3 lanzar o ficheiro launch como se indica no manual do Turtlebot3 https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/
    Para conectar o mando executar: _rosrun joy joy\_node_
    Controlador de mando: _python3 controladorMando.py_
    Prodeceder a executar os algoritmos de aprendizaxe dende o pc remoto
    A imaxe en tempo real da cámara pode visualizarse con _rqt_

Repositorio de probas

    https://github.com/xan-limia/tfg_xan_RL_externo_probas

