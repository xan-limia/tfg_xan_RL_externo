#!/bin/bash
programa="python3 ./qlearnig.py"

if [ $# -eq 0 ]; then
    echo "Uso: $0 <probas a executar>"
    exit 1
fi

# numero de probas a executar
n=$1

#executar n veces e reiniciar a posición dos modelos do simulador
for ((i=1; i<=$n; i++))
do
    echo "Proba $i"
    rosservice call /gazebo/reset_simulation "{}"
    $programa "prueba_auto_ql_d120_m3000_$i+1"
done
