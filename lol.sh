# USER_ID=$(id -u)
# GROUP_ID=$(id -g)

 docker run --rm  -v .:/tmp/CloudCompare:z -t cc-dev:latest ls -l /tmp/CloudCompare
 #--mount type=bind,src=.,dst=/CloudCompare