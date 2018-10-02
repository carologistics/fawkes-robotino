#!/bin/bash

SCRIPT_PATH=$(dirname $(readlink -f ${BASH_SOURCE[0]}))

# For now, we simply use the Fawkes Buildkite pipeline. We do need to adapt
# the script paths. Therefore, we use sed to adapt the script.
#
# Additional steps can be added at the end by outputting additional steps.
# Note that these jobs will be appended at the end, that is in the build job
# section. Add a wait step to wait for the completion of the builds.

sed -e 's|\.buildkite/|fawkes/.buildkite/|g' $SCRIPT_PATH/../fawkes/.buildkite/pipeline.yml

# Example to add more steps:
# Note, that the steps must be properly indented as if it were
# prefixed with "steps:"!
# cat $SCRIPT_PATH/steps.yml

