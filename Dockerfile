FROM docker.pkg.github.com/carologistics/carologistics-ros/carologistics-ros

COPY . /workdir
WORKDIR /workdir
RUN bash -i -c "make -j$(nproc) all gui && find . -name \".objs_*\" -o -name \".deps_*\" -prune -exec rm -rf '{}' \;"
ENTRYPOINT ["bash",  "-i"]
CMD ["./bin/gazsim.bash", "-x", "start", "-o", "-n", "3", "-r", "-a", "-k", "--mongodb", "--no-refbox", "--terminal=tmux", "--wait"]

