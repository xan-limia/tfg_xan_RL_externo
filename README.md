# tfg_xan_RL_externo

Traballo Fin de Grao 2023 

Uso de coñecemento externo na aprendizaxe por reforzo dun comportamento visual en robótica móvil 

Xan Limia García


\section{Guia de usuario}
A continuación detállase como poñer en funcionamento o noso sistema.

Os requisitos iniciais son ter descargada a imaxe Docker e clonar o repositorio principal.

\begin{itemize}
    \item \textit{doker pull xanlimia/turtlebot3\_simulation}
    \item \textit{git clone git@github.com:xan-limia/tfg\_xan\_RL\_externo.git}
\end{itemize}

\subsection{Execución do contorno de simulación}

\subsubsection{Windows}
Será preciso ter instalado \acrfull{wsl} cunha imaxe de Ubuntu 20.04

\begin{enumerate}
    \item Inciar Docker Desktop.
    \item Iniciar Xming. Importante activar a opción \textit{No acces control}. Resto vai por defecto.
    \item Dende \acrshort{wsl} executar o ficheiro \textit{xan\_1.sh}.
\end{enumerate}

\subsubsection{Linux (Ubuntu 20.04)}

\begin{enumerate}
    \item Inciar docker.
    \item Executar: \textit{xhost +}.
    \item Executar o ficheiro \textit{xan\_1.sh}.
\end{enumerate}


\subsection{Execución do código de aprendizaxe}

\begin{itemize}
    \item Executar: \textit{catkin\_make} 
    \item Lanzar os ficheiros launch situados na carpeta \textit{/src/tb3\_implement/launch/}
    \item Algoritmo de acción supervisada: \textit{python3 reforzoAccionManual.py <folder\_name>}
    \item Algoritmo de qlearning: \textit{python3 qlearning.py <folder\_name>}
    \item Creación de estados iniciais con \textit{off policy: python3 off\_policy.py <bag\_file.bag>}
    \item Funcións de análise no ficheiro \textit{analisis.py}
\end{itemize}

\subsection{Conexión co Turtlebot3 real}
Só dispoñible para a versión 20.04 de ubuntu. Será preciso ter configurado o robot seguindo os pasos da guía de inicio rápido \cite{turtlebot_manual}.

\begin{enumerate}
    \item Executar en PC remoto: \textit{roscore}
    \item En Turtlebot3 lanzar o ficheiro launch como se indica no manual \cite{turtlebot_manual}
    \item Para conectar o mando executar:\textit{ rosrun joy joy\_node}
    \item Controlador de mando: \textit{python3 controladorMando.py}
    \item Prodeceder a executar os algoritmos de aprendizaxe dende o pc remoto
    \item A imaxe en tempo real da cámara pode visualizarse con \textit{rqt}.
\end{enumerate}
