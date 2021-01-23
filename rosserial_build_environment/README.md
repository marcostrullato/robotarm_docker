The idea is to build an environment to compile and upload the firmware to the Arduino controlling the arm. 
The Dockerfile will put everything together, once done you just have to fire the command.sh in /tmp. 
You have to identify the tty to communicate with Arduino, and pass it to docker.
For example:
 docker run -it --rm --device=/dev/ttyACM0 --entrypoint /bin/bash ecdb610b19b1
