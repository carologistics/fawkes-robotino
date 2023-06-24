FROM quay.io/fawkesrobotics/fawkes-builder:f37
COPY . /workdir
WORKDIR /workdir
RUN /bin/bash -l -c "mkdir build; cd build; cmake .. --preset central-agent-ros2 ; make -j$(nproc)"
ENTRYPOINT [ "/bin/bash", "-l", "-c" ]
CMD ["echo HI"]

