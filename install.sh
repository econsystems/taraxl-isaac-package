#!/bin/bash

echo -n 'Enter the absolute path of Isaac SDK( Eg : /home/nvidia/isaac)  : '
read input


while [[ "${input}" == "" ]] || [[ ! -d "${input}" ]] 
do

if [ "${input}" != "" ]
then
echo -e ${COLOR_RED}"[error] ${COLOR_DEFAULT}Invalid path: ${input}"
echo ""
fi

read -p "Enter the absolute path of Isaac SDK( Eg : /home/nvidia/isaac)  : " input
done

cp -r packages/ ${input}/.
cp -r apps/ ${input}/.
cp -r third_party/ ${input}/.
cp WORKSPACE ${input}/.
