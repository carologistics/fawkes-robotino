FROM quay.io/fawkesrobotics/fawkes-builder:f37
COPY . /workdir
WORKDIR /workdir
RUN /bin/bash -c "source /etc/profile; mkdir build; cd build; cmake .. --preset central-agent-ros1 ; make -j$(nproc)"
ENTRYPOINT [ "/bin/bash", "-l", "-c" ]
CMD ["echo HI"]

