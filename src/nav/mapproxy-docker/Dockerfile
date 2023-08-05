#--------- Generic stuff all our Dockerfiles should start with so we get caching ------------
FROM python:2.7
MAINTAINER Daniel Snider<danielsnider12@gmail.com>

RUN apt-get -y update

#-------------Application Specific Stuff ----------------------------------------------------

RUN apt-get install -y \
    python-imaging \
    python-yaml \
    libproj0 \
    libgeos-dev \
    python-lxml \
    libgdal-dev \
    build-essential \
    python-dev \
    libjpeg-dev \
    zlib1g-dev \
    libfreetype6-dev \
    python-virtualenv
RUN pip install Shapely Pillow MapProxy uwsgi

EXPOSE 8080

ADD mapproxy.yaml /mapproxy.yaml
ADD start.sh /start.sh
RUN chmod 0755 /start.sh

CMD /start.sh
