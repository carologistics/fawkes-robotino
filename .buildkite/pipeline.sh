#!/bin/bash

SCRIPT_PATH=$(dirname $(readlink -f ${BASH_SOURCE[0]}))

# For now, we simply use the Fawkes Buildkite pipeline. We use sed to
# slightly modify the script:
# - adapt the script path to point to the fawkes submodule scripts
# - add the fawkes-robotino deploy SSH key env var to the docker call
#
# Additional steps can be added at the end by outputting additional steps.
# Note that these jobs will be appended at the end, that is in the build job
# section. Add a wait step to wait for the completion of the builds.

cat $SCRIPT_PATH/pipeline.yml

# Example to add more steps:
# Note, that the steps must be properly indented as if it were
# prefixed with "steps:"!
# cat $SCRIPT_PATH/steps.yml

