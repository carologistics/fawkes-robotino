FROM quay.io/fawkesrobotics/fawkes-builder:f39-ros2
COPY . /workdir
WORKDIR /workdir
ENV FAWKES_DIR=/workdir
RUN /bin/bash -l -c "mkdir build; cd build; cmake .. --preset central-agent ; make -j$(nproc)"
ENTRYPOINT [ "/bin/bash", "-l", "-c" ]
CMD ["echo HI"]

