#!/bin/bash
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 2 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU Library General Public License for more details.
#
# Read the full text in the LICENSE.GPL file in the doc directory.

SCRIPT_PATH=$(dirname $(readlink -f ${BASH_SOURCE[0]}))

# For now, we simply use the Fawkes Buildkite pipeline. We use sed to
# slightly modify the script:
# - adapt the script path to point to the fawkes submodule scripts
# - add the fawkes-robotino deploy SSH key env var to the docker call
#
# Additional steps can be added at the end by outputting additional steps.
# Note that these jobs will be appended at the end, that is in the build job
# section. Add a wait step to wait for the completion of the builds.

sed $SCRIPT_PATH/../fawkes/.buildkite/pipeline.yml \
	-e 's|\.buildkite/|fawkes/.buildkite/|g' \
	-e 's|run: fawkes|run: fawkes-robotino |g' \
	-e 's/SSH_DEPLOY_PRIVKEY_COMMITTERS/SSH_DEPLOY_PRIVKEY_COMMITTERS\n            - SSH_DEPLOY_PRIVKEY_FAWKES_ROBOTINO/g'

# Example to add more steps:
# Note, that the steps must be properly indented as if it were
# prefixed with "steps:"!
# cat $SCRIPT_PATH/steps.yml
