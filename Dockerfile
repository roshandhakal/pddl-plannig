FROM nvidia/cudagl:11.1-devel-ubuntu20.04

ENV VIRTUALGL_VERSION 2.5.2
ARG NUM_BUILD_CORES


# Install all apt dependencies
RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y tzdata
RUN apt-get update && apt-get install -y software-properties-common
# Add ppa for python3.8 install
RUN apt-add-repository -y ppa:deadsnakes/ppa
RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
	curl ca-certificates cmake git python3.8 python3.8-dev \
	xvfb libxv1 libxrender1 libxrender-dev g++ libgeos-dev \
	libboost-all-dev libcgal-dev ffmpeg python3.8-tk \
	texlive texlive-latex-extra dvipng cm-super \
	libeigen3-dev


# Install VirtualGL
RUN curl -sSL https://downloads.sourceforge.net/project/virtualgl/"${VIRTUALGL_VERSION}"/virtualgl_"${VIRTUALGL_VERSION}"_amd64.deb -o virtualgl_"${VIRTUALGL_VERSION}"_amd64.deb && \
	dpkg -i virtualgl_*_amd64.deb && \
	/opt/VirtualGL/bin/vglserver_config -config +s +f -t && \
	rm virtualgl_*_amd64.deb

#Install Python Dependencies 
RUN curl https://bootstrap.pypa.io/get-pip.py -o get-pip.py && python3 get-pip.py && rm get-pip.py
COPY requirements.txt requirements.txt
RUN pip3 install -r requirements.txt

# Install PDDLStream
RUN git clone https://github.com/caelan/pddlstream.git \
	&& cd pddlstream \
	&& ls -a && cat .gitmodules\
	&& sed -i 's/ss-pybullet/pybullet-planning/' .gitmodules \
	&& sed -i 's/git@github.com:caelan\/downward.git/https:\/\/github.com\/caelan\/downward/' .gitmodules \
	&& git submodule update --init --recursive
RUN cd pddlstream\
	&& ./downward/build.py
ENV PYTHONPATH="/pddlstream:${PYTHONPATH}"

# Install Motion Planners by Caelan
RUN git clone https://github.com/caelan/motion-planners.git 
ENV PYTHONPATH="/motion-planners:${PYTHONPATH}"


COPY plan plan
RUN pip3 install plan
# Set up the starting point for running the code
COPY entrypoint.sh /entrypoint.sh
RUN chmod 755 /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
