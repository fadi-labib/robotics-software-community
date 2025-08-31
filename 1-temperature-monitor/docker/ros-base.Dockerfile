FROM ros:humble

# setup environment
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y \
    python3-pip \
    build-essential \
    && rm -rf /var/lib/apt/lists/*

# Create a user 
RUN useradd -ms /bin/bash rosuser
USER rosuser
WORKDIR /home/rosuser
