# Webots-Docker

[![Dockerhub](https://img.shields.io/docker/automated/cyberbotics/webots.svg)](https://hub.docker.com/r/cyberbotics/webots)
[![Test](https://github.com/cyberbotics/webots-docker/workflows/Test/badge.svg)](https://github.com/cyberbotics/webots-docker/actions?query=workflow%3ATest)
[![Docker Image CI](https://github.com/cyberbotics/webots-docker/workflows/Docker%20Image%20CI/badge.svg)](https://github.com/cyberbotics/webots-docker/actions?query=workflow%3A%22Docker+Image+CI%22)

EDIT FRA OLIVER: Hallo Daniel :). Jeg har bare klonet dokumentasjonen til dette imaget på Docker hub: https://hub.docker.com/r/cyberbotics/webots. For å starte webots inne i containeren, måtte jeg kjøre containeren med GPU akselerasjon, måtte da installere nvidia-docker2. Det er fordi jeg er på Linux Ubuntu. Har ikke satt meg inn i hvordan det funker på Windows, men skal funke greit med desktop og litt veivisning fra chatgpt. Dokumentasjonen til webots om docker bruk, dekker bare for linux. Linken til Webots dokumentasjon er den første linken du ser under.


For å builde docker filen kjører jeg
``` bash
Skal sette inn kommando senere
```


Jeg åpner hele kontaineren på denne måten
``` bash
Skal sette inn kommando senere
```


This repository is used to create a Docker image with Webots already pre-installed.
To use the already available image please follow the [Webots installation instructions](https://cyberbotics.com/doc/guide/installation-procedure#installing-the-docker-image).


## Build the Image

Use the following command to build the docker container from the Dockerfile:

``` bash
docker build . --file Dockerfile --tag cyberbotics/webots:latest --build-arg WEBOTS_PACKAGE_PREFIX=_ubuntu-22.04
```

## Build the Webots.Cloud Images

Use the following command to build the docker container from the Dockerfile_webots_cloud:

``` bash
docker build . --file Dockerfile_webots_cloud --tag cyberbotics/webots.cloud:latest
```

## Run a Docker container from the Image

You can run the previously built image with:

``` bash
docker run -it cyberbotics/webots:latest /bin/bash
```

## Clean the temporary Images

You can run the following command to remove **all** temporary images:

``` bash
docker system prune
```




