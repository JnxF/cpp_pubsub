docker-machine start
docker-machine env
eval $(docker-machine env)
docker build -t docker_image_cpp_pubsub .
docker run -it docker_image_cpp_pubsub